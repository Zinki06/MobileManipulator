#include "aruco_localizer/route_config.hpp"
#include "aruco_localizer/marker_guidance.hpp"
#include "aruco_localizer/msg/marker_observation.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

namespace aruco_localizer
{

class ArucoWaypointNavigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Spin = nav2_msgs::action::Spin;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;
  using BackUp = nav2_msgs::action::BackUp;
  using GoalHandleBackUp = rclcpp_action::ClientGoalHandle<BackUp>;
  using Trigger = std_srvs::srv::Trigger;

  ArucoWaypointNavigator()
  : Node("aruco_waypoint_navigator")
  {
    declareParameters();
    loadRouteConfig();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, navigate_to_pose_action_);
    spin_action_client_ = rclcpp_action::create_client<Spin>(
      this, spin_action_);
    backup_action_client_ = rclcpp_action::create_client<BackUp>(
      this, backup_action_);

    marker_observation_sub_ =
      this->create_subscription<aruco_localizer::msg::MarkerObservation>(
      "/aruco/marker_observation", rclcpp::SensorDataQoS(),
      std::bind(
        &ArucoWaypointNavigator::markerObservationCallback, this,
        std::placeholders::_1));
    correction_marker_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/aruco/last_marker_id", 10,
      std::bind(
        &ArucoWaypointNavigator::correctionMarkerCallback, this,
        std::placeholders::_1));

    event_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/route_navigation/events", 50);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/route_navigation/status", rclcpp::QoS(1).transient_local().reliable());
    path_visual_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/route_navigation/path", rclcpp::QoS(1).transient_local().reliable());
    expected_marker_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "/aruco/expected_marker_id", rclcpp::QoS(1).transient_local().reliable());

    nav_to_0_service_ = this->create_service<Trigger>(
      "/navigate_to_marker_0",
      std::bind(
        &ArucoWaypointNavigator::handleNavTo0, this,
        std::placeholders::_1, std::placeholders::_2));
    nav_to_5_service_ = this->create_service<Trigger>(
      "/navigate_to_marker_5",
      std::bind(
        &ArucoWaypointNavigator::handleNavTo5, this,
        std::placeholders::_1, std::placeholders::_2));
    start_patrol_service_ = this->create_service<Trigger>(
      "/start_marker_patrol",
      std::bind(
        &ArucoWaypointNavigator::handleStartPatrol, this,
        std::placeholders::_1, std::placeholders::_2));
    start_station_scan_service_ = this->create_service<Trigger>(
      "/start_station_scan_test",
      std::bind(
        &ArucoWaypointNavigator::handleStartStationScan, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_patrol_service_ = this->create_service<Trigger>(
      "/stop_marker_patrol",
      std::bind(
        &ArucoWaypointNavigator::handleStop, this,
        std::placeholders::_1, std::placeholders::_2));

    visual_timer_ = this->create_wall_timer(
      1s, std::bind(&ArucoWaypointNavigator::publishRouteVisual, this));

    publishStatus();
    emitEvent(
      "NODE_INIT",
      "Virtual waypoint navigator initialized; ArUco visibility is not a route gate.");
  }

private:
  struct MarkerSighting
  {
    double bearing{0.0};
    double planar_distance{0.0};
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
  };

  enum class ExecutionState
  {
    IDLE,
    SENDING_GOAL,
    NAVIGATING,
    SENDING_SPIN,
    SPINNING,
    SCAN_SETTLING,
    MARKER_REACQUIRING,
    SENDING_BACKUP,
    BACKING_UP,
    RETRY_WAIT,
    CANCELING
  };

  void declareParameters()
  {
    this->declare_parameter<std::string>("route_yaml_path", "");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>(
      "navigate_to_pose_action", "/navigate_to_pose");
    this->declare_parameter<std::string>("spin_action", "/spin");
    this->declare_parameter<std::string>("backup_action", "/backup");
    this->declare_parameter<int>("max_navigation_retries", 2);
    this->declare_parameter<int>("retry_cooldown_ms", 500);
    this->declare_parameter<int>("scan_turn_count", 4);
    this->declare_parameter<double>(
      "scan_turn_angle", -1.5707963267948966);
    this->declare_parameter<double>("spin_command_scale", 1.20);
    this->declare_parameter<int>("scan_settle_ms", 700);
    this->declare_parameter<double>("scan_turn_timeout", 30.0);
    this->declare_parameter<double>("marker_observation_timeout", 0.8);
    this->declare_parameter<double>("marker_alignment_tolerance", 0.08);
    this->declare_parameter<double>(
      "marker_search_step", -0.2617993877991494);
    this->declare_parameter<int>("marker_search_max_steps", 24);
    this->declare_parameter<int>("marker_alignment_max_attempts", 6);
    this->declare_parameter<int>("marker_confirmation_count", 3);
    this->declare_parameter<int>("marker_recheck_ms", 350);
    this->declare_parameter<double>("marker_reacquire_timeout", 45.0);
    this->declare_parameter<int>("marker_search_steps_before_backup", 3);
    this->declare_parameter<int>("marker_correction_wait_cycles", 2);
    this->declare_parameter<int>("marker_backup_max_attempts", 2);
    this->declare_parameter<double>("marker_backup_distance", 0.12);
    this->declare_parameter<double>("marker_backup_speed", 0.04);
    this->declare_parameter<double>("marker_backup_timeout", 10.0);
    this->declare_parameter<double>("recovery_arrival_tolerance", 0.15);

    route_yaml_path_ = this->get_parameter("route_yaml_path").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    navigate_to_pose_action_ =
      this->get_parameter("navigate_to_pose_action").as_string();
    spin_action_ = this->get_parameter("spin_action").as_string();
    backup_action_ = this->get_parameter("backup_action").as_string();
    max_navigation_retries_ =
      this->get_parameter("max_navigation_retries").as_int();
    retry_cooldown_ms_ = this->get_parameter("retry_cooldown_ms").as_int();
    scan_turn_count_ = this->get_parameter("scan_turn_count").as_int();
    scan_turn_angle_ = this->get_parameter("scan_turn_angle").as_double();
    spin_command_scale_ = this->get_parameter("spin_command_scale").as_double();
    scan_settle_ms_ = this->get_parameter("scan_settle_ms").as_int();
    scan_turn_timeout_ = this->get_parameter("scan_turn_timeout").as_double();
    marker_observation_timeout_ =
      this->get_parameter("marker_observation_timeout").as_double();
    marker_alignment_tolerance_ =
      this->get_parameter("marker_alignment_tolerance").as_double();
    marker_search_step_ = this->get_parameter("marker_search_step").as_double();
    marker_search_max_steps_ =
      this->get_parameter("marker_search_max_steps").as_int();
    marker_alignment_max_attempts_ =
      this->get_parameter("marker_alignment_max_attempts").as_int();
    marker_confirmation_count_ =
      this->get_parameter("marker_confirmation_count").as_int();
    marker_recheck_ms_ = this->get_parameter("marker_recheck_ms").as_int();
    marker_reacquire_timeout_ =
      this->get_parameter("marker_reacquire_timeout").as_double();
    marker_search_steps_before_backup_ =
      this->get_parameter("marker_search_steps_before_backup").as_int();
    marker_correction_wait_cycles_ =
      this->get_parameter("marker_correction_wait_cycles").as_int();
    marker_backup_max_attempts_ =
      this->get_parameter("marker_backup_max_attempts").as_int();
    marker_backup_distance_ =
      this->get_parameter("marker_backup_distance").as_double();
    marker_backup_speed_ =
      this->get_parameter("marker_backup_speed").as_double();
    marker_backup_timeout_ =
      this->get_parameter("marker_backup_timeout").as_double();
    recovery_arrival_tolerance_ =
      this->get_parameter("recovery_arrival_tolerance").as_double();

    if (max_navigation_retries_ < 0) {
      throw std::invalid_argument("max_navigation_retries must be non-negative");
    }
    if (retry_cooldown_ms_ < 1) {
      throw std::invalid_argument("retry_cooldown_ms must be positive");
    }
    if (recovery_arrival_tolerance_ <= 0.0) {
      throw std::invalid_argument("recovery_arrival_tolerance must be positive");
    }
    if (scan_turn_count_ < 1 || scan_turn_count_ > 16) {
      throw std::invalid_argument("scan_turn_count must be between 1 and 16");
    }
    if (!std::isfinite(scan_turn_angle_) || std::abs(scan_turn_angle_) < 0.01) {
      throw std::invalid_argument("scan_turn_angle must be finite and non-zero");
    }
    if (!std::isfinite(spin_command_scale_) || spin_command_scale_ < 0.5 ||
      spin_command_scale_ > 2.0)
    {
      throw std::invalid_argument("spin_command_scale must be between 0.5 and 2.0");
    }
    if (scan_settle_ms_ < 0) {
      throw std::invalid_argument("scan_settle_ms must be non-negative");
    }
    if (!std::isfinite(scan_turn_timeout_) || scan_turn_timeout_ <= 0.0) {
      throw std::invalid_argument("scan_turn_timeout must be finite and positive");
    }
    if (marker_observation_timeout_ <= 0.0 ||
      marker_alignment_tolerance_ <= 0.0 ||
      !std::isfinite(marker_search_step_) ||
      std::abs(marker_search_step_) < 0.01 ||
      marker_search_max_steps_ < 1 || marker_alignment_max_attempts_ < 1 ||
      marker_confirmation_count_ < 1 || marker_recheck_ms_ < 1 ||
      marker_reacquire_timeout_ <= 0.0)
    {
      throw std::invalid_argument("Invalid marker reacquisition parameter range");
    }
    if (marker_search_steps_before_backup_ < 1 ||
      marker_correction_wait_cycles_ < 1 || marker_backup_max_attempts_ < 0 ||
      !std::isfinite(marker_backup_distance_) || marker_backup_distance_ < 0.05 ||
      marker_backup_distance_ > 0.30 || !std::isfinite(marker_backup_speed_) ||
      marker_backup_speed_ <= 0.0 || marker_backup_speed_ > 0.15 ||
      !std::isfinite(marker_backup_timeout_) || marker_backup_timeout_ <= 0.0)
    {
      throw std::invalid_argument("Invalid marker backup recovery parameter range");
    }
  }

  void loadRouteConfig()
  {
    if (route_yaml_path_.empty()) {
      config_error_ = "route_yaml_path parameter is empty";
      RCLCPP_ERROR(this->get_logger(), "%s", config_error_.c_str());
      return;
    }

    try {
      route_config_ = RouteConfig::loadFromFile(route_yaml_path_);
      config_loaded_ = true;
      RCLCPP_INFO(
        this->get_logger(), "Loaded virtual waypoint routes from %s",
        route_yaml_path_.c_str());
    } catch (const std::exception & exception) {
      config_error_ = exception.what();
      RCLCPP_ERROR(
        this->get_logger(), "Failed to load route config %s: %s",
        route_yaml_path_.c_str(), exception.what());
    }
  }

  bool routeActive() const
  {
    return state_ != ExecutionState::IDLE;
  }

  std::string stateName() const
  {
    switch (state_) {
      case ExecutionState::IDLE:
        return "IDLE";
      case ExecutionState::SENDING_GOAL:
        return "SENDING_GOAL";
      case ExecutionState::NAVIGATING:
        return "NAVIGATING";
      case ExecutionState::SENDING_SPIN:
        return "SENDING_SPIN";
      case ExecutionState::SPINNING:
        return "SPINNING";
      case ExecutionState::SCAN_SETTLING:
        return "SCAN_SETTLING";
      case ExecutionState::MARKER_REACQUIRING:
        return "MARKER_REACQUIRING";
      case ExecutionState::SENDING_BACKUP:
        return "SENDING_BACKUP";
      case ExecutionState::BACKING_UP:
        return "BACKING_UP";
      case ExecutionState::RETRY_WAIT:
        return "RETRY_WAIT";
      case ExecutionState::CANCELING:
        return "CANCELING";
    }
    return "UNKNOWN";
  }

  geometry_msgs::msg::PoseStamped poseForWaypoint(
    const RouteWaypoint & waypoint) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = this->now();
    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, waypoint.yaw);
    pose.pose.orientation = tf2::toMsg(quaternion);
    return pose;
  }

  bool beginRoute(
    const std::string & route_name, const std::string & patrol_return,
    std::string & message, bool scan_at_each_waypoint = false)
  {
    if (!config_loaded_) {
      message = "Cannot start: route config is invalid: " + config_error_;
      return false;
    }
    if (routeActive()) {
      message = "Another navigation task is already active.";
      return false;
    }
    if (!route_config_.hasRoute(route_name)) {
      message = "Unknown route: " + route_name;
      return false;
    }
    if (scan_at_each_waypoint) {
      for (const auto & waypoint_name : route_config_.route(route_name)) {
        if (route_config_.waypoint(waypoint_name).marker_id < 0) {
          message = "Cannot start scan test: waypoint '" + waypoint_name +
            "' has no marker_id.";
          return false;
        }
      }
    }

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (!getRobotPose(x, y, yaw)) {
      message = "Cannot start: map to base pose is not initialized.";
      return false;
    }
    (void)x;
    (void)y;
    (void)yaw;

    if (!nav_action_client_->wait_for_action_server(3s)) {
      message = "Cannot start: Nav2 navigate_to_pose action is unavailable.";
      return false;
    }
    if (scan_at_each_waypoint && !spin_action_client_->wait_for_action_server(3s)) {
      message = "Cannot start scan test: Nav2 spin action is unavailable.";
      return false;
    }
    if (scan_at_each_waypoint && !backup_action_client_->wait_for_action_server(3s)) {
      message = "Cannot start scan test: Nav2 backup action is unavailable.";
      return false;
    }

    active_route_name_ = route_name;
    active_waypoint_names_ = route_config_.route(route_name);
    pending_patrol_route_ = patrol_return;
    scan_at_each_waypoint_ = scan_at_each_waypoint;
    waypoint_index_ = 0U;
    retry_count_ = 0;
    scan_turn_index_ = 0;
    marker_search_steps_ = 0;
    marker_alignment_attempts_ = 0;
    marker_search_steps_since_backup_ = 0;
    marker_backup_attempts_ = 0;
    marker_correction_wait_count_ = 0;
    marker_reacquisition_active_ = false;
    state_ = ExecutionState::SENDING_GOAL;

    emitEvent(
      "ROUTE_START",
      "Starting virtual route '" + route_name + "' (mode=" +
      (scan_at_each_waypoint_ ? "station_scan" : "navigate") + ").");
    sendCurrentGoal();
    message = "Started virtual waypoint route '" + route_name + "'.";
    return true;
  }

  void sendCurrentGoal()
  {
    if (!routeActive() || waypoint_index_ >= active_waypoint_names_.size()) {
      completeRoute();
      return;
    }

    const std::string & waypoint_name = active_waypoint_names_[waypoint_index_];
    const RouteWaypoint & waypoint = route_config_.waypoint(waypoint_name);
    publishExpectedMarker(scan_at_each_waypoint_ ? waypoint.marker_id : -1);
    NavigateToPose::Goal goal;
    goal.pose = poseForWaypoint(waypoint);

    const uint64_t goal_id = ++current_goal_id_;
    state_ = ExecutionState::SENDING_GOAL;
    publishStatus();

    std::ostringstream details;
    details << "Sending waypoint " << (waypoint_index_ + 1U) << "/" <<
      active_waypoint_names_.size() << " '" << waypoint.name << "' pose=(" <<
      waypoint.x << ", " << waypoint.y << ", " << waypoint.yaw <<
      " rad), role=" << waypoint.role << ", goal_id=" << goal_id;
    emitEvent("WAYPOINT_START", details.str());

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this, goal_id](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
        goalResponseCallback(goal_handle, goal_id);
      };
    options.feedback_callback =
      [this, goal_id](
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        feedbackCallback(feedback, goal_id);
      };
    options.result_callback =
      [this, goal_id](const GoalHandleNavigateToPose::WrappedResult & result) {
        resultCallback(result, goal_id);
      };
    nav_action_client_->async_send_goal(goal, options);
  }

  void publishExpectedMarker(int marker_id)
  {
    if (expected_marker_id_ == marker_id) {
      return;
    }
    expected_marker_id_ = marker_id;
    expected_marker_observation_count_ = 0;
    last_expected_marker_observation_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    std_msgs::msg::Int32 message;
    message.data = marker_id;
    expected_marker_pub_->publish(message);
    emitEvent(
      "EXPECTED_MARKER_CHANGED",
      marker_id < 0 ? "Accepting any known marker for localization." :
      "Only marker ID " + std::to_string(marker_id) +
      " may correct localization; other IDs remain search hints.");
  }

  void goalResponseCallback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle, uint64_t goal_id)
  {
    if (!routeActive() || goal_id != current_goal_id_) {
      if (goal_handle) {
        nav_action_client_->async_cancel_goal(goal_handle);
      }
      return;
    }

    if (state_ == ExecutionState::CANCELING) {
      if (!goal_handle) {
        finishAbort(pending_abort_reason_);
        return;
      }
      active_goal_handle_ = goal_handle;
      nav_action_client_->async_cancel_goal(goal_handle);
      emitEvent(
        "NAV_CANCEL_REQUESTED",
        "Canceling goal_id=" + std::to_string(goal_id) + " after late acceptance.");
      return;
    }

    if (!goal_handle) {
      handleGoalFailure("Nav2 rejected the goal", goal_id);
      return;
    }

    active_goal_handle_ = goal_handle;
    state_ = ExecutionState::NAVIGATING;
    publishStatus();
    emitEvent("NAV_ACCEPTED", "Nav2 accepted goal_id=" + std::to_string(goal_id));
  }

  void feedbackCallback(
    const std::shared_ptr<const NavigateToPose::Feedback> & feedback,
    uint64_t goal_id)
  {
    if (!routeActive() || goal_id != current_goal_id_) {
      return;
    }
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "[NAV_FEEDBACK] route=%s waypoint=%s remaining=%.3fm recoveries=%d",
      active_route_name_.c_str(), currentWaypointName().c_str(),
      feedback->distance_remaining, feedback->number_of_recoveries);
  }

  void resultCallback(
    const GoalHandleNavigateToPose::WrappedResult & result, uint64_t goal_id)
  {
    if (goal_id != current_goal_id_) {
      RCLCPP_DEBUG(
        this->get_logger(), "Ignored stale result for goal %lu (current %lu)",
        goal_id, current_goal_id_);
      return;
    }

    active_goal_handle_.reset();
    if (!routeActive()) {
      return;
    }

    if (state_ == ExecutionState::CANCELING) {
      finishAbort(pending_abort_reason_);
      return;
    }

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      emitEvent(
        "WAYPOINT_REACHED",
        "Nav2 completed waypoint '" + currentWaypointName() + "'.");
      retry_count_ = 0;
      if (scan_at_each_waypoint_) {
        startWaypointScan();
      } else {
        advanceWaypoint();
      }
      return;
    }

    if (result.code == rclcpp_action::ResultCode::ABORTED &&
      robotIsWithinRecoveryTolerance())
    {
      emitEvent(
        "WAYPOINT_RECOVERY_REACHED",
        "Nav2 aborted after reaching the configured recovery tolerance for '" +
        currentWaypointName() + "'.");
      retry_count_ = 0;
      if (scan_at_each_waypoint_) {
        startWaypointScan();
      } else {
        advanceWaypoint();
      }
      return;
    }

    const std::string result_name =
      result.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "ABORTED";
    handleGoalFailure("Nav2 result=" + result_name, goal_id);
  }

  void advanceWaypoint()
  {
    ++waypoint_index_;
    scan_turn_index_ = 0;
    sendCurrentGoal();
  }

  void startWaypointScan()
  {
    scan_turn_index_ = 0;
    emitEvent(
      "STATION_SCAN_START",
      "Starting 360-degree scan at station " +
      std::to_string(waypoint_index_) + " ('" + currentWaypointName() +
      "'); " + std::to_string(scan_turn_count_) + " right turns planned.");
    sendSpinGoal();
  }

  void sendSpinGoal()
  {
    if (!routeActive() || !scan_at_each_waypoint_ ||
      scan_turn_index_ >= scan_turn_count_)
    {
      return;
    }

    Spin::Goal goal;
    const double commanded_turn = scan_turn_angle_ * spin_command_scale_;
    goal.target_yaw = static_cast<float>(commanded_turn);
    goal.time_allowance.sec = static_cast<int32_t>(scan_turn_timeout_);
    goal.time_allowance.nanosec = static_cast<uint32_t>(
      (scan_turn_timeout_ - static_cast<double>(goal.time_allowance.sec)) *
      1000000000.0);

    const uint64_t goal_id = ++current_goal_id_;
    state_ = ExecutionState::SENDING_SPIN;
    publishStatus();
    emitEvent(
      "SCAN_TURN_START",
      "Station " + std::to_string(waypoint_index_) + " turn " +
      std::to_string(scan_turn_index_ + 1) + "/" +
      std::to_string(scan_turn_count_) + ", desired_yaw=" +
      std::to_string(scan_turn_angle_) + " rad, command_yaw=" +
      std::to_string(commanded_turn) + " rad, scale=" +
      std::to_string(spin_command_scale_) + ", goal_id=" +
      std::to_string(goal_id));

    auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
    options.goal_response_callback =
      [this, goal_id](const GoalHandleSpin::SharedPtr & goal_handle) {
        spinGoalResponseCallback(goal_handle, goal_id);
      };
    options.feedback_callback =
      [this, goal_id](
      GoalHandleSpin::SharedPtr,
      const std::shared_ptr<const Spin::Feedback> feedback) {
        if (!routeActive() || goal_id != current_goal_id_) {
          return;
        }
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[SCAN_FEEDBACK] station=%zu turn=%d/%d traveled=%.3frad",
          waypoint_index_, scan_turn_index_ + 1, scan_turn_count_,
          feedback->angular_distance_traveled);
      };
    options.result_callback =
      [this, goal_id](const GoalHandleSpin::WrappedResult & result) {
        spinResultCallback(result, goal_id);
      };
    spin_action_client_->async_send_goal(goal, options);
  }

  void spinGoalResponseCallback(
    const GoalHandleSpin::SharedPtr & goal_handle, uint64_t goal_id)
  {
    if (!routeActive() || goal_id != current_goal_id_) {
      if (goal_handle) {
        spin_action_client_->async_cancel_goal(goal_handle);
      }
      return;
    }

    if (state_ == ExecutionState::CANCELING) {
      if (!goal_handle) {
        finishAbort(pending_abort_reason_);
        return;
      }
      active_spin_goal_handle_ = goal_handle;
      spin_action_client_->async_cancel_goal(goal_handle);
      return;
    }

    if (!goal_handle) {
      handleSpinFailure("Nav2 rejected the spin goal", goal_id);
      return;
    }

    active_spin_goal_handle_ = goal_handle;
    state_ = ExecutionState::SPINNING;
    publishStatus();
  }

  void spinResultCallback(
    const GoalHandleSpin::WrappedResult & result, uint64_t goal_id)
  {
    if (goal_id != current_goal_id_) {
      return;
    }
    active_spin_goal_handle_.reset();
    if (!routeActive()) {
      return;
    }
    if (state_ == ExecutionState::CANCELING) {
      finishAbort(pending_abort_reason_);
      return;
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      const std::string result_name =
        result.code == rclcpp_action::ResultCode::CANCELED ?
        "CANCELED" : "ABORTED";
      handleSpinFailure("Nav2 spin result=" + result_name, goal_id);
      return;
    }

    ++scan_turn_index_;
    state_ = ExecutionState::SCAN_SETTLING;
    publishStatus();
    emitEvent(
      "SCAN_TURN_COMPLETE",
      "Station " + std::to_string(waypoint_index_) + " completed turn " +
      std::to_string(scan_turn_index_) + "/" +
      std::to_string(scan_turn_count_) + "; waiting " +
      std::to_string(scan_settle_ms_) + "ms before capture-ready event.");

    scan_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(scan_settle_ms_),
      [this, goal_id]() {
        scan_timer_->cancel();
        if (!routeActive() || goal_id != current_goal_id_ ||
          state_ != ExecutionState::SCAN_SETTLING)
        {
          return;
        }
        emitEvent(
          "SCAN_HEADING_READY",
          "Station " + std::to_string(waypoint_index_) + " heading " +
          std::to_string(scan_turn_index_) + "/" +
          std::to_string(scan_turn_count_) + " is stationary and ready for capture.");
        if (scan_turn_index_ >= scan_turn_count_) {
          emitEvent(
            "STATION_SCAN_COMPLETE",
            "Completed 360-degree scan at station " +
            std::to_string(waypoint_index_) + " ('" +
            currentWaypointName() + "').");
          if (waypoint_index_ + 1U < active_waypoint_names_.size()) {
            startNextMarkerReacquisition();
          } else {
            advanceWaypoint();
          }
        } else {
          sendSpinGoal();
        }
      });
  }

  void handleSpinFailure(const std::string & reason, uint64_t goal_id)
  {
    if (!routeActive() || goal_id != current_goal_id_) {
      return;
    }
    active_spin_goal_handle_.reset();
    finishAbort(reason + "; shared motion failed safely; see /motion/events");
  }

  void markerObservationCallback(
    const aruco_localizer::msg::MarkerObservation::SharedPtr message)
  {
    MarkerSighting sighting;
    sighting.bearing = message->bearing;
    sighting.planar_distance = message->planar_distance;
    sighting.received_at = this->now();
    marker_sightings_[message->marker_id] = sighting;
    if (message->marker_id == expected_marker_id_) {
      if (last_expected_marker_observation_at_.nanoseconds() == 0 ||
        (sighting.received_at - last_expected_marker_observation_at_).seconds() >
        0.25)
      {
        expected_marker_observation_count_ = 1;
      } else {
        ++expected_marker_observation_count_;
      }
      last_expected_marker_observation_at_ = sighting.received_at;
    }
  }

  void correctionMarkerCallback(const std_msgs::msg::Int32::SharedPtr message)
  {
    last_correction_marker_id_ = message->data;
    last_correction_received_at_ = this->now();
  }

  bool freshMarkerSighting(int marker_id, MarkerSighting & sighting) const
  {
    const auto iterator = marker_sightings_.find(marker_id);
    if (iterator == marker_sightings_.end()) {
      return false;
    }
    if ((this->now() - iterator->second.received_at).seconds() >
      marker_observation_timeout_)
    {
      return false;
    }
    sighting = iterator->second;
    return true;
  }

  void startNextMarkerReacquisition()
  {
    if (waypoint_index_ + 1U >= active_waypoint_names_.size()) {
      advanceWaypoint();
      return;
    }
    const RouteWaypoint & next_waypoint = route_config_.waypoint(
      active_waypoint_names_[waypoint_index_ + 1U]);
    if (next_waypoint.marker_id < 0) {
      finishAbort("Next scan station has no marker_id for visual reacquisition");
      return;
    }

    marker_reacquisition_active_ = true;
    marker_target_id_ = next_waypoint.marker_id;
    marker_search_steps_ = 0;
    marker_alignment_attempts_ = 0;
    marker_search_steps_since_backup_ = 0;
    marker_backup_attempts_ = 0;
    marker_correction_wait_count_ = 0;
    marker_reacquisition_started_at_ = this->now();
    state_ = ExecutionState::MARKER_REACQUIRING;
    publishExpectedMarker(marker_target_id_);
    emitEvent(
      "MARKER_REACQUIRE_START",
      "Before leaving station " + std::to_string(waypoint_index_) +
      ", searching for next marker ID " + std::to_string(marker_target_id_) +
      ". Non-target markers are guidance only and cannot correct map->odom.");
    scheduleMarkerEvaluation(marker_recheck_ms_);
  }

  void scheduleMarkerEvaluation(int delay_ms)
  {
    state_ = ExecutionState::MARKER_REACQUIRING;
    publishStatus();
    const uint64_t operation_id = current_goal_id_;
    marker_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(delay_ms),
      [this, operation_id]() {
        marker_timer_->cancel();
        if (!routeActive() || !marker_reacquisition_active_ ||
          operation_id != current_goal_id_ ||
          state_ != ExecutionState::MARKER_REACQUIRING)
        {
          return;
        }
        evaluateMarkerReacquisition();
      });
  }

  bool correctionIsFreshForTarget() const
  {
    return last_correction_marker_id_ == marker_target_id_ &&
           last_correction_received_at_.nanoseconds() != 0 &&
           (this->now() - last_correction_received_at_).seconds() <=
           marker_observation_timeout_;
  }

  void evaluateMarkerReacquisition()
  {
    if ((this->now() - marker_reacquisition_started_at_).seconds() >
      marker_reacquire_timeout_)
    {
      continueWithOdometry(
        "Timed out while reacquiring next marker ID " +
        std::to_string(marker_target_id_));
      return;
    }

    MarkerSighting target_sighting;
    if (freshMarkerSighting(marker_target_id_, target_sighting)) {
      if (std::abs(target_sighting.bearing) <= marker_alignment_tolerance_ &&
        expected_marker_observation_count_ >= marker_confirmation_count_)
      {
        if (correctionIsFreshForTarget()) {
          emitEvent(
            "MARKER_REACQUIRED",
            "Next marker ID " + std::to_string(marker_target_id_) +
            " is centered at bearing=" +
            std::to_string(target_sighting.bearing * 180.0 / M_PI) +
            "deg and has corrected localization.");
          marker_reacquisition_active_ = false;
          advanceWaypoint();
          return;
        }
        ++marker_correction_wait_count_;
        if (marker_correction_wait_count_ >= marker_correction_wait_cycles_) {
          if (startMarkerBackup(
              "Marker is centered but localization correction is unavailable; "
              "it may be too close or clipped"))
          {
            return;
          }
          continueWithOdometry(
            "Expected marker stayed centered but could not provide a valid correction");
          return;
        }
        emitEvent(
          "MARKER_ALIGNMENT_WAIT",
          "Marker ID " + std::to_string(marker_target_id_) +
          " is centered; waiting briefly for consistent localization correction (" +
          std::to_string(marker_correction_wait_count_) + "/" +
          std::to_string(marker_correction_wait_cycles_) + ").");
        scheduleMarkerEvaluation(marker_recheck_ms_);
        return;
      }

      marker_correction_wait_count_ = 0;
      sendMarkerTurn(
        target_sighting.bearing, false,
        "Centering visible next marker ID " +
        std::to_string(marker_target_id_));
      return;
    }

    if (tryGeometryGuidedMarkerTurn()) {
      return;
    }

    if (marker_search_steps_ >= marker_search_max_steps_) {
      continueWithOdometry(
        "Completed the marker search sweep without finding next marker ID " +
        std::to_string(marker_target_id_));
      return;
    }

    if (marker_search_steps_since_backup_ >= marker_search_steps_before_backup_) {
      if (startMarkerBackup(
          "Short symmetric search did not reveal the complete expected marker"))
      {
        return;
      }
      continueWithOdometry(
        "Expected marker remained unavailable after bounded search and backup recovery");
      return;
    }
    const double search_turn = aruco_localizer::symmetricSearchTurn(
      marker_search_step_, marker_search_steps_since_backup_);
    sendMarkerTurn(
      search_turn, true,
      "No complete expected marker visible; performing bounded symmetric search");
  }

  bool startMarkerBackup(const std::string & reason)
  {
    if (marker_backup_attempts_ >= marker_backup_max_attempts_) {
      return false;
    }
    if (!backup_action_client_->action_server_is_ready()) {
      emitEvent(
        "MARKER_BACKUP_UNAVAILABLE",
        "Nav2 backup action is unavailable; cannot perform marker recovery.");
      return false;
    }

    ++marker_backup_attempts_;
    marker_search_steps_since_backup_ = 0;
    marker_alignment_attempts_ = 0;
    marker_correction_wait_count_ = 0;
    expected_marker_observation_count_ = 0;
    last_expected_marker_observation_at_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    BackUp::Goal goal;
    goal.target.x = -marker_backup_distance_;
    goal.speed = static_cast<float>(marker_backup_speed_);
    goal.time_allowance.sec = static_cast<int32_t>(marker_backup_timeout_);
    goal.time_allowance.nanosec = static_cast<uint32_t>(
      (marker_backup_timeout_ - static_cast<double>(goal.time_allowance.sec)) *
      1000000000.0);

    const uint64_t goal_id = ++current_goal_id_;
    state_ = ExecutionState::SENDING_BACKUP;
    emitEvent(
      "MARKER_BACKUP_START",
      reason + "; backing up " + std::to_string(marker_backup_distance_) +
      "m at " + std::to_string(marker_backup_speed_) + "m/s (attempt " +
      std::to_string(marker_backup_attempts_) + "/" +
      std::to_string(marker_backup_max_attempts_) + ", goal_id=" +
      std::to_string(goal_id) + ").");

    auto options = rclcpp_action::Client<BackUp>::SendGoalOptions();
    options.goal_response_callback =
      [this, goal_id](const GoalHandleBackUp::SharedPtr & goal_handle) {
        markerBackupGoalResponseCallback(goal_handle, goal_id);
      };
    options.result_callback =
      [this, goal_id](const GoalHandleBackUp::WrappedResult & result) {
        markerBackupResultCallback(result, goal_id);
      };
    backup_action_client_->async_send_goal(goal, options);
    return true;
  }

  void markerBackupGoalResponseCallback(
    const GoalHandleBackUp::SharedPtr & goal_handle, uint64_t goal_id)
  {
    if (!routeActive() || goal_id != current_goal_id_) {
      if (goal_handle) {
        backup_action_client_->async_cancel_goal(goal_handle);
      }
      return;
    }
    if (state_ == ExecutionState::CANCELING) {
      if (!goal_handle) {
        finishAbort(pending_abort_reason_);
        return;
      }
      active_backup_goal_handle_ = goal_handle;
      backup_action_client_->async_cancel_goal(goal_handle);
      return;
    }
    if (!goal_handle) {
      continueWithOdometry("Nav2 rejected marker backup recovery");
      return;
    }
    active_backup_goal_handle_ = goal_handle;
    state_ = ExecutionState::BACKING_UP;
    publishStatus();
  }

  void markerBackupResultCallback(
    const GoalHandleBackUp::WrappedResult & result, uint64_t goal_id)
  {
    if (goal_id != current_goal_id_) {
      return;
    }
    active_backup_goal_handle_.reset();
    if (!routeActive()) {
      return;
    }
    if (state_ == ExecutionState::CANCELING) {
      finishAbort(pending_abort_reason_);
      return;
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      continueWithOdometry(
        "Marker backup was blocked or failed; preserving frozen map-to-odom");
      return;
    }

    emitEvent(
      "MARKER_BACKUP_COMPLETE",
      "Backup recovery completed; checking the expected marker again.");
    scheduleMarkerEvaluation(scan_settle_ms_);
  }

  void continueWithOdometry(const std::string & reason)
  {
    emitEvent(
      "MARKER_REACQUIRE_DEGRADED",
      reason + "; continuing to the next station with the frozen map-to-odom "
      "transform and wheel odometry.");
    marker_reacquisition_active_ = false;
    advanceWaypoint();
  }

  bool tryGeometryGuidedMarkerTurn()
  {
    const RouteWaypoint & current_station = route_config_.waypoint(
      active_waypoint_names_[waypoint_index_]);
    const RouteWaypoint & target_marker =
      route_config_.waypointForMarker(marker_target_id_);
    const double target_map_bearing = std::atan2(
      target_marker.y - current_station.y,
      target_marker.x - current_station.x);

    int guidance_marker_id = -1;
    MarkerSighting guidance_sighting;
    if (waypoint_index_ > 0U) {
      const RouteWaypoint & previous_station = route_config_.waypoint(
        active_waypoint_names_[waypoint_index_ - 1U]);
      if (previous_station.marker_id >= 0 &&
        freshMarkerSighting(previous_station.marker_id, guidance_sighting))
      {
        guidance_marker_id = previous_station.marker_id;
      }
    }

    if (guidance_marker_id < 0) {
      for (const auto & entry : marker_sightings_) {
        if (!route_config_.hasMarkerWaypoint(entry.first) ||
          entry.first == marker_target_id_ ||
          (this->now() - entry.second.received_at).seconds() >
          marker_observation_timeout_)
        {
          continue;
        }
        const RouteWaypoint & observed_marker =
          route_config_.waypointForMarker(entry.first);
        if (std::hypot(
            observed_marker.x - current_station.x,
            observed_marker.y - current_station.y) < 0.10)
        {
          continue;
        }
        guidance_marker_id = entry.first;
        guidance_sighting = entry.second;
        break;
      }
    }

    if (guidance_marker_id < 0) {
      return false;
    }

    const RouteWaypoint & guidance_marker =
      route_config_.waypointForMarker(guidance_marker_id);
    const double guidance_map_bearing = std::atan2(
      guidance_marker.y - current_station.y,
      guidance_marker.x - current_station.x);
    const double map_direction_change = aruco_localizer::normalizeAngle(
      target_map_bearing - guidance_map_bearing);
    const double predicted_target_bearing = aruco_localizer::predictTargetBearing(
      guidance_sighting.bearing, guidance_map_bearing, target_map_bearing);

    if (std::abs(predicted_target_bearing) <= marker_alignment_tolerance_) {
      return false;
    }
    if (marker_alignment_attempts_ >= marker_alignment_max_attempts_) {
      return false;
    }

    const bool is_previous = waypoint_index_ > 0U &&
      route_config_.waypoint(active_waypoint_names_[waypoint_index_ - 1U]).marker_id ==
      guidance_marker_id;
    std::ostringstream details;
    details << (is_previous ? "Previous" : "Known") << " marker ID " <<
      guidance_marker_id << " seen at " <<
      guidance_sighting.bearing * 180.0 / M_PI <<
      "deg; route geometry predicts next marker ID " << marker_target_id_ <<
      " at " << predicted_target_bearing * 180.0 / M_PI << "deg";
    if (std::abs(std::abs(map_direction_change) - M_PI) < 0.35) {
      details << " (approximately 180deg opposite)";
    }
    emitEvent("MARKER_GEOMETRY_GUIDANCE", details.str());
    sendMarkerTurn(predicted_target_bearing, false, details.str());
    return true;
  }

  void sendMarkerTurn(
    double desired_turn, bool is_search, const std::string & reason)
  {
    if (is_search) {
      ++marker_search_steps_;
      ++marker_search_steps_since_backup_;
    } else {
      if (marker_alignment_attempts_ >= marker_alignment_max_attempts_) {
        sendMarkerTurn(
          marker_search_step_, true,
          "Alignment attempts exhausted; resuming bounded search");
        return;
      }
      ++marker_alignment_attempts_;
    }
    current_marker_desired_turn_ = desired_turn;
    current_marker_turn_is_search_ = is_search;
    current_marker_turn_reason_ = reason;
    sendMarkerTurnGoal();
  }

  void sendMarkerTurnGoal()
  {
    Spin::Goal goal;
    const double commanded_turn =
      current_marker_desired_turn_ * spin_command_scale_;
    goal.target_yaw = static_cast<float>(commanded_turn);
    goal.time_allowance.sec = static_cast<int32_t>(scan_turn_timeout_);
    goal.time_allowance.nanosec = static_cast<uint32_t>(
      (scan_turn_timeout_ - static_cast<double>(goal.time_allowance.sec)) *
      1000000000.0);

    const uint64_t goal_id = ++current_goal_id_;
    state_ = ExecutionState::SENDING_SPIN;
    publishStatus();
    emitEvent(
      current_marker_turn_is_search_ ?
      "MARKER_SEARCH_TURN_START" : "MARKER_ALIGN_TURN_START",
      current_marker_turn_reason_ + "; desired=" +
      std::to_string(current_marker_desired_turn_ * 180.0 / M_PI) +
      "deg command=" + std::to_string(commanded_turn * 180.0 / M_PI) +
      "deg goal_id=" + std::to_string(goal_id));

    auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
    options.goal_response_callback =
      [this, goal_id](const GoalHandleSpin::SharedPtr & goal_handle) {
        markerSpinGoalResponseCallback(goal_handle, goal_id);
      };
    options.result_callback =
      [this, goal_id](const GoalHandleSpin::WrappedResult & result) {
        markerSpinResultCallback(result, goal_id);
      };
    spin_action_client_->async_send_goal(goal, options);
  }

  void markerSpinGoalResponseCallback(
    const GoalHandleSpin::SharedPtr & goal_handle, uint64_t goal_id)
  {
    if (!routeActive() || goal_id != current_goal_id_) {
      if (goal_handle) {
        spin_action_client_->async_cancel_goal(goal_handle);
      }
      return;
    }
    if (state_ == ExecutionState::CANCELING) {
      if (!goal_handle) {
        finishAbort(pending_abort_reason_);
        return;
      }
      active_spin_goal_handle_ = goal_handle;
      spin_action_client_->async_cancel_goal(goal_handle);
      return;
    }
    if (!goal_handle) {
      handleMarkerSpinFailure("Nav2 rejected marker-search spin", goal_id);
      return;
    }
    active_spin_goal_handle_ = goal_handle;
    state_ = ExecutionState::SPINNING;
    publishStatus();
  }

  void markerSpinResultCallback(
    const GoalHandleSpin::WrappedResult & result, uint64_t goal_id)
  {
    if (goal_id != current_goal_id_) {
      return;
    }
    active_spin_goal_handle_.reset();
    if (!routeActive()) {
      return;
    }
    if (state_ == ExecutionState::CANCELING) {
      finishAbort(pending_abort_reason_);
      return;
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      handleMarkerSpinFailure("Marker-search Spin did not succeed", goal_id);
      return;
    }
    scheduleMarkerEvaluation(scan_settle_ms_);
  }

  void handleMarkerSpinFailure(const std::string & reason, uint64_t goal_id)
  {
    if (!routeActive() || goal_id != current_goal_id_) {
      return;
    }
    active_spin_goal_handle_.reset();
    finishAbort(reason + "; shared motion failed safely; no odometry fallback after motion failure");
  }

  void handleGoalFailure(const std::string & reason, uint64_t goal_id)
  {
    if (!routeActive() || goal_id != current_goal_id_) {
      return;
    }

    active_goal_handle_.reset();
    if (retry_count_ >= max_navigation_retries_) {
      finishAbort(reason + " after all retries");
      return;
    }

    ++retry_count_;
    state_ = ExecutionState::RETRY_WAIT;
    publishStatus();
    emitEvent(
      "NAV_RETRY",
      reason + "; retry " + std::to_string(retry_count_) + "/" +
      std::to_string(max_navigation_retries_) + " after " +
      std::to_string(retry_cooldown_ms_) + "ms.");

    const size_t expected_index = waypoint_index_;
    retry_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(retry_cooldown_ms_),
      [this, goal_id, expected_index]() {
        retry_timer_->cancel();
        if (!routeActive() || goal_id != current_goal_id_ ||
        expected_index != waypoint_index_ || state_ != ExecutionState::RETRY_WAIT)
        {
          return;
        }
        sendCurrentGoal();
      });
  }

  bool robotIsWithinRecoveryTolerance()
  {
    if (waypoint_index_ >= active_waypoint_names_.size()) {
      return false;
    }

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (!getRobotPose(x, y, yaw)) {
      emitEvent(
        "TF_UNAVAILABLE",
        "Could not evaluate recovery arrival because map to base TF is unavailable.");
      return false;
    }
    (void)yaw;

    const RouteWaypoint & waypoint =
      route_config_.waypoint(active_waypoint_names_[waypoint_index_]);
    return std::hypot(x - waypoint.x, y - waypoint.y) <=
           recovery_arrival_tolerance_;
  }

  void completeRoute()
  {
    emitEvent("ROUTE_COMPLETE", "Completed virtual route '" + active_route_name_ + "'.");

    if (!pending_patrol_route_.empty()) {
      active_route_name_ = pending_patrol_route_;
      pending_patrol_route_.clear();
      active_waypoint_names_ = route_config_.route(active_route_name_);
      waypoint_index_ = 0U;
      retry_count_ = 0;
      state_ = ExecutionState::SENDING_GOAL;
      emitEvent(
        "PATROL_RETURN", "Starting return route '" + active_route_name_ + "'.");
      sendCurrentGoal();
      return;
    }

    state_ = ExecutionState::IDLE;
    active_route_name_.clear();
    active_waypoint_names_.clear();
    scan_at_each_waypoint_ = false;
    marker_reacquisition_active_ = false;
    marker_backup_attempts_ = 0;
    waypoint_index_ = 0U;
    scan_turn_index_ = 0;
    publishExpectedMarker(-1);
    publishStatus();
  }

  void requestRouteCancellation(const std::string & reason)
  {
    if (!routeActive()) {
      return;
    }

    pending_abort_reason_ = reason;
    if (retry_timer_) {
      retry_timer_->cancel();
    }
    if (scan_timer_) {
      scan_timer_->cancel();
    }
    if (marker_timer_) {
      marker_timer_->cancel();
    }
    if (state_ == ExecutionState::RETRY_WAIT ||
      state_ == ExecutionState::SCAN_SETTLING ||
      state_ == ExecutionState::MARKER_REACQUIRING)
    {
      finishAbort(reason);
      return;
    }

    state_ = ExecutionState::CANCELING;
    emitEvent(
      "NAV_CANCEL_REQUESTED",
      "Waiting for Nav2 to finish canceling the active route goal.");
    if (active_goal_handle_) {
      nav_action_client_->async_cancel_goal(active_goal_handle_);
    }
    if (active_spin_goal_handle_) {
      spin_action_client_->async_cancel_goal(active_spin_goal_handle_);
    }
    if (active_backup_goal_handle_) {
      backup_action_client_->async_cancel_goal(active_backup_goal_handle_);
    }
    // When a goal is still being sent, goalResponseCallback cancels it as soon
    // as Nav2 returns the handle. IDLE is published only after its final result.
  }

  void finishAbort(const std::string & reason)
  {
    const std::string route_name = active_route_name_;
    ++current_goal_id_;
    active_goal_handle_.reset();
    active_spin_goal_handle_.reset();
    active_backup_goal_handle_.reset();

    state_ = ExecutionState::IDLE;
    active_route_name_.clear();
    active_waypoint_names_.clear();
    pending_patrol_route_.clear();
    pending_abort_reason_.clear();
    scan_at_each_waypoint_ = false;
    marker_reacquisition_active_ = false;
    waypoint_index_ = 0U;
    retry_count_ = 0;
    scan_turn_index_ = 0;
    publishExpectedMarker(-1);
    emitEvent("ROUTE_ABORTED", "Route '" + route_name + "': " + reason);
    publishStatus();
  }

  bool getRobotPose(double & x, double & y, double & yaw)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.1));
      x = transform.transform.translation.x;
      y = transform.transform.translation.y;
      yaw = tf2::getYaw(transform.transform.rotation);
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Map pose unavailable: %s", exception.what());
      return false;
    }
  }

  std::string currentWaypointName() const
  {
    if (waypoint_index_ >= active_waypoint_names_.size()) {
      return "none";
    }
    return active_waypoint_names_[waypoint_index_];
  }

  void emitEvent(const std::string & tag, const std::string & details)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] %s", tag.c_str(), details.c_str());
    if (event_pub_) {
      std_msgs::msg::String event;
      event.data = tag + "|" + details;
      event_pub_->publish(event);
    }
    publishStatus();
  }

  void publishStatus()
  {
    if (!status_pub_) {
      return;
    }
    std_msgs::msg::String status;
    std::ostringstream value;
    value << "state=" << stateName() << ";route=" <<
      (active_route_name_.empty() ? "none" : active_route_name_) <<
      ";waypoint=" << currentWaypointName() << ";index=" << waypoint_index_ <<
      ";count=" << active_waypoint_names_.size() << ";retry=" << retry_count_ <<
      ";mode=" << (scan_at_each_waypoint_ ? "station_scan" : "navigate") <<
      ";scan_turn=" << scan_turn_index_ << "/" << scan_turn_count_ <<
      ";expected_marker=" << expected_marker_id_ <<
      ";marker_search_steps=" << marker_search_steps_ <<
      ";marker_backup_attempts=" << marker_backup_attempts_;
    status.data = value.str();
    status_pub_->publish(status);
  }

  void handleNavTo0(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    response->success = beginRoute("to_0", "", response->message);
  }

  void handleNavTo5(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    response->success = beginRoute("to_5", "", response->message);
  }

  void handleStartPatrol(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (!getRobotPose(x, y, yaw)) {
      response->success = false;
      response->message = "Cannot start patrol without a current map pose.";
      return;
    }
    (void)x;
    (void)yaw;

    const bool start_toward_zero = y > 1.0;
    response->success = beginRoute(
      start_toward_zero ? "to_0" : "to_5",
      start_toward_zero ? "to_5" : "to_0", response->message);
  }

  void handleStartStationScan(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    response->success = beginRoute(
      "station_scan_0_to_5", "", response->message, true);
  }

  void handleStop(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (!routeActive()) {
      response->success = false;
      response->message = "No active navigation task.";
      return;
    }
    requestRouteCancellation("Stop service requested");
    response->success = true;
    response->message = "Navigation stop requested.";
  }

  void publishRouteVisual()
  {
    if (!config_loaded_) {
      return;
    }

    const std::string route_name =
      active_route_name_.empty() ? "to_5" : active_route_name_;
    if (!route_config_.hasRoute(route_name)) {
      return;
    }

    visualization_msgs::msg::Marker line;
    line.header.frame_id = map_frame_;
    line.header.stamp = this->now();
    line.ns = "virtual_route";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.04;
    line.color.r = 0.1F;
    line.color.g = 0.9F;
    line.color.b = 0.3F;
    line.color.a = 0.9F;

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (getRobotPose(robot_x, robot_y, robot_yaw)) {
      geometry_msgs::msg::Point start;
      start.x = robot_x;
      start.y = robot_y;
      line.points.push_back(start);
    }

    for (const auto & waypoint_name : route_config_.route(route_name)) {
      const RouteWaypoint & waypoint = route_config_.waypoint(waypoint_name);
      geometry_msgs::msg::Point point;
      point.x = waypoint.x;
      point.y = waypoint.y;
      line.points.push_back(point);
    }
    path_visual_pub_->publish(line);
  }

  std::string route_yaml_path_;
  std::string map_frame_;
  std::string base_frame_;
  std::string navigate_to_pose_action_;
  std::string spin_action_;
  std::string backup_action_;
  int max_navigation_retries_{2};
  int retry_cooldown_ms_{500};
  int scan_turn_count_{4};
  double scan_turn_angle_{-1.5707963267948966};
  double spin_command_scale_{1.20};
  int scan_settle_ms_{700};
  double scan_turn_timeout_{30.0};
  double marker_observation_timeout_{0.8};
  double marker_alignment_tolerance_{0.08};
  double marker_search_step_{-0.2617993877991494};
  int marker_search_max_steps_{24};
  int marker_alignment_max_attempts_{6};
  int marker_confirmation_count_{3};
  int marker_recheck_ms_{350};
  double marker_reacquire_timeout_{45.0};
  int marker_search_steps_before_backup_{3};
  int marker_correction_wait_cycles_{2};
  int marker_backup_max_attempts_{2};
  double marker_backup_distance_{0.12};
  double marker_backup_speed_{0.04};
  double marker_backup_timeout_{10.0};
  double recovery_arrival_tolerance_{0.15};

  RouteConfig route_config_;
  bool config_loaded_{false};
  std::string config_error_;
  ExecutionState state_{ExecutionState::IDLE};
  std::string active_route_name_;
  std::vector<std::string> active_waypoint_names_;
  std::string pending_patrol_route_;
  std::string pending_abort_reason_;
  size_t waypoint_index_{0U};
  int retry_count_{0};
  int scan_turn_index_{0};
  bool scan_at_each_waypoint_{false};
  bool marker_reacquisition_active_{false};
  int expected_marker_id_{-2};
  int marker_target_id_{-1};
  int expected_marker_observation_count_{0};
  int last_correction_marker_id_{-1};
  int marker_search_steps_{0};
  int marker_search_steps_since_backup_{0};
  int marker_backup_attempts_{0};
  int marker_correction_wait_count_{0};
  int marker_alignment_attempts_{0};
  bool current_marker_turn_is_search_{false};
  double current_marker_desired_turn_{0.0};
  std::string current_marker_turn_reason_;
  rclcpp::Time marker_reacquisition_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_correction_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_expected_marker_observation_at_{0, 0, RCL_ROS_TIME};
  std::unordered_map<int, MarkerSighting> marker_sightings_;
  uint64_t current_goal_id_{0U};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  GoalHandleNavigateToPose::SharedPtr active_goal_handle_;
  rclcpp_action::Client<Spin>::SharedPtr spin_action_client_;
  GoalHandleSpin::SharedPtr active_spin_goal_handle_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_action_client_;
  GoalHandleBackUp::SharedPtr active_backup_goal_handle_;
  rclcpp::Subscription<aruco_localizer::msg::MarkerObservation>::SharedPtr
    marker_observation_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr correction_marker_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_visual_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr expected_marker_pub_;
  rclcpp::Service<Trigger>::SharedPtr nav_to_0_service_;
  rclcpp::Service<Trigger>::SharedPtr nav_to_5_service_;
  rclcpp::Service<Trigger>::SharedPtr start_patrol_service_;
  rclcpp::Service<Trigger>::SharedPtr start_station_scan_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_patrol_service_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::TimerBase::SharedPtr visual_timer_;
};

}  // namespace aruco_localizer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aruco_localizer::ArucoWaypointNavigator>());
  rclcpp::shutdown();
  return 0;
}
