#include "cleanup_task_manager/task_config.hpp"
#include "cleanup_task_manager/navigation_evidence.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <cleanup_interfaces/msg/object_observation.hpp>
#include <cleanup_interfaces/srv/capture_objects.hpp>
#include <cleanup_interfaces/srv/plan_cleanup.hpp>
#include <cleanup_interfaces/srv/evaluate_grasp.hpp>
#include <cleanup_interfaces/srv/place_object.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <unistd.h>

using namespace std::chrono_literals;

namespace cleanup_task_manager
{

class CleanupTaskManager : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Spin = nav2_msgs::action::Spin;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;
  using CaptureObjects = cleanup_interfaces::srv::CaptureObjects;
  using PlanCleanup = cleanup_interfaces::srv::PlanCleanup;
  using EvaluateGrasp = cleanup_interfaces::srv::EvaluateGrasp;
  using PlaceObject = cleanup_interfaces::srv::PlaceObject;
  using ObjectObservation = cleanup_interfaces::msg::ObjectObservation;
  using Trigger = std_srvs::srv::Trigger;

  CleanupTaskManager()
  : Node("cleanup_task_manager")
  {
    declareParameters();
    loadConfig();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    navigation_evidence_ = std::make_unique<NavigationEvidence>(*this);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, navigate_action_);
    spin_client_ = rclcpp_action::create_client<Spin>(this, spin_action_);
    capture_client_ = this->create_client<CaptureObjects>(
      "/cleanup/capture_objects");
    planner_client_ = this->create_client<PlanCleanup>(
      "/cleanup/plan_objects");
    require_candidate_grasp_ = declare_parameter<bool>("require_candidate_grasp", false);
    pick_client_ = this->create_client<Trigger>("/execute_pick_and_place");
    grasp_check_client_ = this->create_client<EvaluateGrasp>("/cleanup/evaluate_grasp");
    open_gripper_client_ = this->create_client<Trigger>("/open_gripper");
    place_client_ = this->create_client<PlaceObject>("/cleanup/place_object");
    park_arm_client_ = this->create_client<Trigger>("/park_arm");
    observe_floor_client_ = this->create_client<Trigger>("/observe_floor");

    localization_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/aruco/localization_mode", 10,
      std::bind(
        &CleanupTaskManager::localizationModeCallback, this,
        std::placeholders::_1));
    motion_guard_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/motion_guard/status", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        motion_guard_status_ = msg->data;
        guard_received_ = std::chrono::steady_clock::now();
        if (missionActive() && msg->data.rfind("FAULT:", 0) == 0) {
          failMission("Motion safety fault: " + msg->data);
        }
      });
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/cleanup/status", rclcpp::QoS(1).transient_local().reliable());
    event_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/cleanup/events", 50);
    pick_target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/cleanup/pick_target", 10);
    start_service_ = this->create_service<Trigger>(
      "/start_cleanup",
      std::bind(
        &CleanupTaskManager::handleStart, this,
        std::placeholders::_1, std::placeholders::_2));
    stop_service_ = this->create_service<Trigger>(
      "/stop_cleanup",
      std::bind(
        &CleanupTaskManager::handleStop, this,
        std::placeholders::_1, std::placeholders::_2));

    publishStatus();
    emitEvent("NODE_INIT", "Station cleanup manager is idle.");
  }

private:
  enum class State
  {
    IDLE,
    PARKING_FOR_SCAN,
    NAVIGATING_TO_STATION,
    NAVIGATION_RETRY_WAIT,
    SCAN_SETTLING,
    SENDING_SPIN,
    SPINNING,
    CAPTURING,
    OBSERVATION_ALIGNING,
    OBSERVATION_SETTLING,
    OBSERVATION_CAPTURING,
    PLANNING,
    NAVIGATING_TO_OBJECT,
    CHECKING_GRASP,
    REACQUIRING_TARGET,
    PICKING,
    VERIFYING_PICK,
    NAVIGATING_TO_DROP,
    RELEASING,
    PARKING_AFTER_DROP,
    RECOVERING_RELEASE,
    RECOVERING_ARM,
    RETURNING_TO_STATION,
    COMPLETE
  };

  enum class NavigationPurpose
  {
    NONE,
    STATION,
    OBJECT_APPROACH,
    DROP_ZONE,
    RETURN_STATION,
  };

  void declareParameters()
  {
    this->declare_parameter<std::string>("task_config_path", "");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>(
      "navigate_to_pose_action", "/navigate_to_pose");
    this->declare_parameter<std::string>("spin_action", "/spin");
    this->declare_parameter<double>("spin_command_scale", 1.0);
    this->declare_parameter<double>("scan_turn_timeout", 30.0);
    this->declare_parameter<int>("max_pick_retries", 1);
    this->declare_parameter<int>("max_approach_attempts", 3);
    max_navigation_retries_ = this->declare_parameter<int>("max_navigation_retries", 2);
    navigation_retry_delay_ = this->declare_parameter<double>("navigation_retry_delay", 2.0);
    navigation_retry_timeout_ = this->declare_parameter<double>("navigation_retry_timeout", 10.0);
    this->declare_parameter<std::string>("approach_behavior_tree", "");
    this->declare_parameter<double>("observation_yaw_tolerance", 0.10);
    this->declare_parameter<int>("observation_max_turns", 3);
    this->declare_parameter<std::string>(
      "debug_output_root", "/home/user/turtlebot3_ws/cleanup_debug");

    task_config_path_ = this->get_parameter("task_config_path").as_string();
    max_approach_attempts_ = this->get_parameter("max_approach_attempts").as_int();
    approach_behavior_tree_ = this->get_parameter("approach_behavior_tree").as_string();
    if (approach_behavior_tree_.empty()) {
      approach_behavior_tree_ = (std::filesystem::path(task_config_path_).parent_path().
        parent_path() / "behavior_trees" / "precision_approach.xml").string();
    }
    map_frame_ = this->get_parameter("map_frame").as_string();
    navigate_action_ =
      this->get_parameter("navigate_to_pose_action").as_string();
    spin_action_ = this->get_parameter("spin_action").as_string();
    spin_command_scale_ = this->get_parameter("spin_command_scale").as_double();
    scan_turn_timeout_ = this->get_parameter("scan_turn_timeout").as_double();
    max_pick_retries_ = this->get_parameter("max_pick_retries").as_int();
    observation_yaw_tolerance_ =
      this->get_parameter("observation_yaw_tolerance").as_double();
    observation_max_turns_ = this->get_parameter("observation_max_turns").as_int();
    debug_output_root_ = this->get_parameter("debug_output_root").as_string();

    if (!std::isfinite(spin_command_scale_) || spin_command_scale_ < 0.5 ||
      spin_command_scale_ > 2.0 || !std::isfinite(scan_turn_timeout_) ||
      scan_turn_timeout_ <= 0.0 ||
      max_pick_retries_ < 0 || max_approach_attempts_ < 1 || max_approach_attempts_ > 6 ||
      max_navigation_retries_ < 0 || max_navigation_retries_ > 3 ||
      !std::isfinite(navigation_retry_delay_) || navigation_retry_delay_ < 0.5 ||
      !std::isfinite(navigation_retry_timeout_) ||
      navigation_retry_timeout_ < navigation_retry_delay_ + 0.5 ||
      navigation_retry_timeout_ > 30.0 ||
      !std::isfinite(observation_yaw_tolerance_) ||
      observation_yaw_tolerance_ <= 0.0 || observation_yaw_tolerance_ > 0.3 ||
      observation_max_turns_ < 1 || observation_max_turns_ > 6)
    {
      throw std::invalid_argument("Invalid cleanup action parameter range");
    }
  }

  void loadConfig()
  {
    if (task_config_path_.empty()) {
      config_error_ = "task_config_path parameter is empty";
      return;
    }
    try {
      config_ = TaskConfig::loadFromFile(task_config_path_);
      config_loaded_ = true;
      RCLCPP_INFO(
        this->get_logger(), "Loaded %zu cleanup stations from %s",
        config_.stations.size(), task_config_path_.c_str());
    } catch (const std::exception & exception) {
      config_error_ = exception.what();
      RCLCPP_ERROR(this->get_logger(), "Task config error: %s", exception.what());
    }
  }

  std::string stateName() const
  {
    switch (state_) {
      case State::IDLE: return "IDLE";
      case State::NAVIGATION_RETRY_WAIT: return "NAVIGATION_RETRY_WAIT";
      case State::PARKING_FOR_SCAN: return "PARKING_FOR_SCAN";
      case State::NAVIGATING_TO_STATION: return "NAVIGATING_TO_STATION";
      case State::SCAN_SETTLING: return "SCAN_SETTLING";
      case State::SENDING_SPIN: return "SENDING_SPIN";
      case State::SPINNING: return "SPINNING";
      case State::CAPTURING: return "CAPTURING";
      case State::OBSERVATION_ALIGNING: return "OBSERVATION_ALIGNING";
      case State::OBSERVATION_SETTLING: return "OBSERVATION_SETTLING";
      case State::OBSERVATION_CAPTURING: return "OBSERVATION_CAPTURING";
      case State::PLANNING: return "PLANNING";
      case State::NAVIGATING_TO_OBJECT: return "NAVIGATING_TO_OBJECT";
      case State::CHECKING_GRASP: return "CHECKING_GRASP";
      case State::REACQUIRING_TARGET: return "REACQUIRING_TARGET";
      case State::PICKING: return "PICKING";
      case State::VERIFYING_PICK: return "VERIFYING_PICK";
      case State::NAVIGATING_TO_DROP: return "NAVIGATING_TO_DROP";
      case State::RELEASING: return "RELEASING";
      case State::PARKING_AFTER_DROP: return "PARKING_AFTER_DROP";
      case State::RECOVERING_RELEASE: return "RECOVERING_RELEASE";
      case State::RECOVERING_ARM: return "RECOVERING_ARM";
      case State::RETURNING_TO_STATION: return "RETURNING_TO_STATION";
      case State::COMPLETE: return "COMPLETE";
    }
    return "UNKNOWN";
  }

  bool missionActive() const
  {
    return state_ != State::IDLE && state_ != State::COMPLETE;
  }

  bool callbackCurrent(uint64_t session, uint64_t operation) const
  {
    return missionActive() && session == mission_session_ &&
           operation == operation_id_;
  }

  void localizationModeCallback(
    const std_msgs::msg::String::SharedPtr message)
  {
    localization_mode_ = message->data;
    localization_received_ = std::chrono::steady_clock::now();
    if (missionActive() && localization_mode_ == "DEGRADED") {
      failMission("Localization became DEGRADED during cleanup.");
    }
  }

  bool dependenciesReady() const
  {
    return nav_client_->action_server_is_ready() &&
           spin_client_->action_server_is_ready() &&
           capture_client_->service_is_ready() &&
           planner_client_->service_is_ready() &&
           pick_client_->service_is_ready() &&
           grasp_check_client_->service_is_ready() &&
           open_gripper_client_->service_is_ready() &&
           place_client_->service_is_ready() &&
           park_arm_client_->service_is_ready() && observe_floor_client_->service_is_ready();
  }

  void handleStart(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (missionActive()) {
      response->success = false;
      response->message = "Cleanup is already active.";
      return;
    }
    if (!config_loaded_) {
      response->success = false;
      response->message = "Invalid task config: " + config_error_;
      return;
    }
    if (!config_.drop_pose_configured) {
      response->success = false;
      response->message = "The cleanup drop pose is not configured.";
      return;
    }
    if (localization_mode_.empty() || localization_mode_ == "UNINITIALIZED" ||
      localization_mode_ == "DEGRADED")
    {
      response->success = false;
      response->message = "Cleanup requires initialized, non-degraded localization.";
      return;
    }
    if (!dependenciesReady()) {
      response->success = false;
      response->message =
        "Cleanup dependencies are not ready (Nav2, perception, planner, or arm).";
      return;
    }
    if (motion_guard_status_.empty() || motion_guard_status_.rfind("FAULT:", 0) == 0) {
      response->success = false;
      response->message = "Motion guard unavailable or faulted; inspect before restart.";
      return;
    }
    if (!std::filesystem::is_regular_file(approach_behavior_tree_)) {
      response->success = false;
      response->message = "Precision approach behavior tree is missing: " + approach_behavior_tree_;
      return;
    }

    ++mission_session_;
    operation_id_ = 0U;
    station_index_ = 0U;
    heading_index_ = 0;
    capture_retry_count_ = 0;
    pick_retry_count_ = 0;
    collected_objects_.clear();
    failed_objects_.clear();
    incomplete_scan_count_ = 0;
    station_observations_.clear();
    reviewed_objects_.clear();
    known_objects_.clear();
    active_object_uuid_.clear();
    gripper_may_hold_object_ = false;
    navigation_purpose_ = NavigationPurpose::NONE;
    startEvidenceSession();
    emitEvent(
      "MISSION_START",
      "Starting 0-to-5 station cleanup; drop zone=" + config_.drop_zone_name +
      ", planner=" + config_.planner + ".");
    requestInitialArmPark();
    response->success = true;
    response->message = "Cleanup mission started: " + mission_id_;
  }

  void handleStop(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (!missionActive()) {
      response->success = false;
      response->message = "Cleanup is not active.";
      return;
    }
    cancelActiveOperations();
    state_ = State::IDLE;
    navigation_purpose_ = NavigationPurpose::NONE;
    emitEvent(
      "MISSION_STOPPED",
      "Cleanup stopped; an already-running arm service cannot be preempted.");
    response->success = true;
    response->message = "Cleanup stop requested.";
  }

  void requestInitialArmPark()
  {
    state_ = State::PARKING_FOR_SCAN;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent("ARM_PARK_START", "Parking the arm before station scanning.");
    park_arm_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this, session, operation](rclcpp::Client<Trigger>::SharedFuture future) {
        if (!callbackCurrent(session, operation) ||
          state_ != State::PARKING_FOR_SCAN)
        {
          return;
        }
        const auto response = future.get();
        if (!response->success) {
          failMission("Initial arm park failed: " + response->message);
          return;
        }
        navigateToCurrentStation(NavigationPurpose::STATION);
      });
  }

  geometry_msgs::msg::PoseStamped poseMessage(const TaskPose & pose) const
  {
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = map_frame_;
    message.header.stamp = this->now();
    message.pose.position.x = pose.x;
    message.pose.position.y = pose.y;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, pose.yaw);
    message.pose.orientation = tf2::toMsg(orientation);
    return message;
  }

  void navigateToCurrentStation(NavigationPurpose purpose)
  {
    if (station_index_ >= config_.stations.size()) {
      completeMission();
      return;
    }
    sendNavigationGoal(poseMessage(config_.stations[station_index_].pose), purpose);
  }

  void sendNavigationGoal(
    const geometry_msgs::msg::PoseStamped & pose, NavigationPurpose purpose,
    bool retry = false)
  {
    if (!retry) {navigation_retry_count_ = 0;}
    last_navigation_pose_ = pose;
    NavigateToPose::Goal goal;
    goal.pose = pose;
    goal.pose.header.stamp = this->now();
    if (purpose == NavigationPurpose::OBJECT_APPROACH || purpose == NavigationPurpose::DROP_ZONE) {
      goal.behavior_tree = approach_behavior_tree_;
    }
    navigation_purpose_ = purpose;
    switch (purpose) {
      case NavigationPurpose::STATION:
        state_ = State::NAVIGATING_TO_STATION;
        break;
      case NavigationPurpose::OBJECT_APPROACH:
        state_ = State::NAVIGATING_TO_OBJECT;
        break;
      case NavigationPurpose::DROP_ZONE:
        state_ = State::NAVIGATING_TO_DROP;
        break;
      case NavigationPurpose::RETURN_STATION:
        state_ = State::RETURNING_TO_STATION;
        break;
      case NavigationPurpose::NONE:
        failMission("Internal error: navigation purpose is NONE.");
        return;
    }

    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent(
      "NAVIGATION_START",
      "purpose=" + navigationPurposeName(purpose) + ", operation=" +
      std::to_string(operation) + ", goal=(" + std::to_string(pose.pose.position.x) +
      "," + std::to_string(pose.pose.position.y) + "), yaw=" +
      std::to_string(tf2::getYaw(pose.pose.orientation)) + ".");
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this, session, operation](const GoalHandleNavigate::SharedPtr & handle) {
        if (!callbackCurrent(session, operation)) {
          if (handle) {
            nav_client_->async_cancel_goal(handle);
          }
          return;
        }
        if (!handle) {
          handleNavigationFailure("Nav2 rejected cleanup goal");
          return;
        }
        active_navigation_goal_ = handle;
      };
    options.result_callback =
      [this, session, operation](const GoalHandleNavigate::WrappedResult & result) {
        if (!callbackCurrent(session, operation)) {
          return;
        }
        active_navigation_goal_.reset();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          const bool aborted = result.code == rclcpp_action::ResultCode::ABORTED;
          handleNavigationFailure(
            aborted ? "Nav2 cleanup goal ABORTED" : "Nav2 cleanup goal canceled/unknown",
            aborted);
          return;
        }
        handleNavigationSuccess();
      };
    nav_client_->async_send_goal(goal, options);
  }

  std::string navigationPurposeName(NavigationPurpose purpose) const
  {
    switch (purpose) {
      case NavigationPurpose::STATION: return "station";
      case NavigationPurpose::OBJECT_APPROACH: return "object";
      case NavigationPurpose::DROP_ZONE: return "drop";
      case NavigationPurpose::RETURN_STATION: return "return_station";
      case NavigationPurpose::NONE: return "none";
    }
    return "unknown";
  }

  void handleNavigationSuccess()
  {
    const NavigationPurpose completed = navigation_purpose_;
    navigation_purpose_ = NavigationPurpose::NONE;
    if (completed == NavigationPurpose::STATION) {
      beginStationScan();
    } else if (completed == NavigationPurpose::OBJECT_APPROACH) {
      requestTargetReacquisition();
    } else if (completed == NavigationPurpose::DROP_ZONE) {
      requestRelease();
    } else if (completed == NavigationPurpose::RETURN_STATION) {
      active_object_uuid_.clear();
      requestPlanForStation();
    }
  }

  geometry_msgs::msg::PoseStamped currentNavigationPose() const
  {
    geometry_msgs::msg::PoseStamped pose;
    try {
      const auto tf = tf_buffer_->lookupTransform(map_frame_, "base_link", tf2::TimePointZero);
      pose.header = tf.header;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
    } catch (const tf2::TransformException &) {
      // Empty frame explicitly records unavailable pose rather than a false origin.
    }
    return pose;
  }

  void recordNavigationFailure(const std::string & reason)
  {
    const auto directory = mission_directory_ /
      ("navigation_failure_" + std::to_string(operation_id_));
    try {
      navigation_evidence_->save(
        directory, last_navigation_pose_, currentNavigationPose(), reason, motion_guard_status_);
      emitEvent("NAVIGATION_FAILURE_EVIDENCE", directory.string());
    } catch (const std::exception & error) {
      emitEvent("NAVIGATION_EVIDENCE_ERROR", error.what());
    }
  }

  std::string navigationRetryBlocker() const
  {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - guard_received_).count() > 0.5) {
      return "motion guard status stale";
    }
    if (motion_guard_status_.empty()) {return "motion guard status missing";}
    // An idle command timeout is expected after Nav2 has stopped. All sensor/TF
    // blocks still prevent restarting, including missing or unknown statuses.
    if (motion_guard_status_ != "READY" &&
      motion_guard_status_ != "BLOCKED: command timeout") {return motion_guard_status_;}
    if (std::chrono::duration<double>(now - localization_received_).count() > 0.5 ||
      localization_mode_.empty() || localization_mode_ == "UNINITIALIZED" ||
      localization_mode_ == "DEGRADED") {return "localization not ready";}
    const auto pose = currentNavigationPose();
    if (pose.header.frame_id.empty()) {return "robot TF unavailable";}
    const double age = (this->now() - rclcpp::Time(pose.header.stamp)).seconds();
    if (age < -0.1 || age > 0.3) {return "robot TF stale";}
    if (!navigation_evidence_->stationary()) {return "odometry stale or robot still moving";}
    if (!nav_client_->action_server_is_ready()) {return "Nav2 unavailable";}
    return "";
  }

  void scheduleNavigationRetry(const std::string & reason)
  {
    ++navigation_retry_count_;
    state_ = State::NAVIGATION_RETRY_WAIT;
    const auto started = std::chrono::steady_clock::now();
    const uint64_t session = mission_session_, operation = ++operation_id_;
    emitEvent("NAVIGATION_RETRY_WAIT", reason + "; purpose=" +
      navigationPurposeName(navigation_purpose_) + "; retry=" +
      std::to_string(navigation_retry_count_) + "/" + std::to_string(max_navigation_retries_));
    // Remain stopped, require 0.5 s of uninterrupted healthy/stationary state.
    navigation_retry_timer_ = this->create_wall_timer(100ms, std::function<void()>(
      [this, session, operation, started, healthy_since = started]() mutable {
        if (!callbackCurrent(session, operation)) {return;}
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - started).count();
        const auto blocker = navigationRetryBlocker();
        if (!blocker.empty()) {healthy_since = now;}
        if (elapsed >= navigation_retry_timeout_) {
          navigation_retry_timer_->cancel();
          failMission("Navigation retry readiness timeout: " +
            (blocker.empty() ? std::string("health not stable") : blocker));
          return;
        }
        if (elapsed >= navigation_retry_delay_ && blocker.empty() &&
          std::chrono::duration<double>(now - healthy_since).count() >= 0.5)
        {
          navigation_retry_timer_->cancel();
          emitEvent("NAVIGATION_RETRY", "Stationary and sensors/TF ready; replan same goal.");
          sendNavigationGoal(last_navigation_pose_, navigation_purpose_, true);
        }
      }));
  }

  void handleNavigationFailure(const std::string & reason, bool retryable = false)
  {
    recordNavigationFailure(reason);
    const NavigationPurpose failed = navigation_purpose_;
    const bool transit = failed == NavigationPurpose::STATION ||
      failed == NavigationPurpose::RETURN_STATION || failed == NavigationPurpose::DROP_ZONE;
    if (retryable && transit && navigation_retry_count_ < max_navigation_retries_) {
      scheduleNavigationRetry(reason);
      return;
    }
    navigation_purpose_ = NavigationPurpose::NONE;
    if (failed == NavigationPurpose::OBJECT_APPROACH) {
      recoverFromObjectFailure(reason + " while approaching object");
      return;
    }
    failMission(reason + " for " + navigationPurposeName(failed));
  }

  void beginStationScan()
  {
    heading_index_ = 0;
    capture_retry_count_ = 0;
    station_observations_.clear();
    reviewed_objects_.clear();
    successful_scan_captures_ = 0;
    scan_correction_count_ = 0;
    double x = 0.0, y = 0.0;
    if (!getRobotPose(x, y, scan_start_yaw_, "odom")) {
      failMission("No robot pose for absolute scan headings");
      return;
    }
    emitEvent(
      "STATION_SCAN_START",
      "station=" + currentStation().name + ", " + std::to_string(config_.scan.turns) +
      "-heading scan starting.");
    scheduleSettle(true);
  }

  void scheduleSettle(bool before_spin)
  {
    state_ = State::SCAN_SETTLING;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    const auto delay = std::chrono::milliseconds(
      static_cast<int64_t>(config_.scan.settle_seconds * 1000.0));
    settle_timer_ = this->create_wall_timer(
      delay,
      [this, session, operation, before_spin]() {
        settle_timer_->cancel();
        if (!callbackCurrent(session, operation) ||
          state_ != State::SCAN_SETTLING)
        {
          return;
        }
        if (before_spin) {
          sendScanSpin();
        } else {
          checkScanHeading();
        }
      });
  }

  void sendScanSpin()
  {
    Spin::Goal goal;
    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!getRobotPose(x, y, yaw, "odom")) {
      failMission("No robot pose for scan rotation");
      return;
    }
    const double target = scan_start_yaw_ +
      (heading_index_ + 1) * config_.scan.turn_angle * spin_command_scale_;
    const double error = std::atan2(std::sin(target - yaw), std::cos(target - yaw));
    const double command = std::clamp(error,
      -3.141592653589793, 3.141592653589793);
    goal.target_yaw = static_cast<float>(command);
    goal.time_allowance.sec = static_cast<int32_t>(scan_turn_timeout_);
    goal.time_allowance.nanosec = static_cast<uint32_t>(
      (scan_turn_timeout_ - static_cast<double>(goal.time_allowance.sec)) *
      1000000000.0);

    state_ = State::SENDING_SPIN;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent(
      "SCAN_TURN_START",
      "station=" + currentStation().name + ", turn=" +
      std::to_string(heading_index_ + 1) + "/" +
      std::to_string(config_.scan.turns) + ", command=" +
      std::to_string(command) + "rad.");
    auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
    options.goal_response_callback =
      [this, session, operation](const GoalHandleSpin::SharedPtr & handle) {
        if (!callbackCurrent(session, operation)) {
          if (handle) {
            spin_client_->async_cancel_goal(handle);
          }
          return;
        }
        if (!handle) {
          handleSpinFailure("Nav2 rejected scan spin");
          return;
        }
        active_spin_goal_ = handle;
        state_ = State::SPINNING;
        publishStatus();
      };
    options.result_callback =
      [this, session, operation](const GoalHandleSpin::WrappedResult & result) {
        if (!callbackCurrent(session, operation)) {
          return;
        }
        active_spin_goal_.reset();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          handleSpinFailure("Nav2 scan spin failed");
          return;
        }
        emitEvent(
          "SCAN_TURN_COMPLETE",
          "station=" + currentStation().name + ", heading=" +
          std::to_string(heading_index_) + " ready after settling.");
        scheduleSettle(false);
      };
    spin_client_->async_send_goal(goal, options);
  }

  void handleSpinFailure(const std::string & reason)
  {
    active_spin_goal_.reset();
    // The shared motion executor already waits for safety and closes residual
    // yaw. A failed turn must not trigger an unrelated map-frame recenter move.
    failMission(reason + "; shared motion failed safely; see /motion/events");
  }

  void checkScanHeading()
  {
    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!getRobotPose(x, y, yaw, "odom")) {
      failMission("Robot pose unavailable after scan turn");
      return;
    }
    const double target = scan_start_yaw_ +
      (heading_index_ + 1) * config_.scan.turn_angle * spin_command_scale_;
    const double error = std::atan2(std::sin(target - yaw), std::cos(target - yaw));
    if (std::abs(error) > observation_yaw_tolerance_) {
      if (++scan_correction_count_ > observation_max_turns_) {
        finishInterruptedScan("Scan heading failed to converge");
        return;
      }
      emitEvent("SCAN_HEADING_CORRECTION", "error=" + std::to_string(error));
      sendScanSpin();
      return;
    }
    requestScanCapture();
  }

  void finishInterruptedScan(const std::string & reason)
  {
    ++incomplete_scan_count_;
    emitEvent("STATION_SCAN_INCOMPLETE", "station=" + currentStation().name + "; " + reason +
      "; retaining observed objects and continuing route; captures=" +
      std::to_string(successful_scan_captures_));
    writeRegistrySnapshot();
    requestPlanForStation();
  }

  std::shared_ptr<CaptureObjects::Request> captureRequest(
    const std::string & station_name, uint8_t heading) const
  {
    auto request = std::make_shared<CaptureObjects::Request>();
    request->mission_id = mission_id_;
    request->station_name = station_name;
    request->heading_index = heading;
    request->burst_frames = static_cast<uint16_t>(config_.scan.burst_frames);
    request->min_confirmations =
      static_cast<uint16_t>(config_.scan.min_confirmations);
    return request;
  }

  void requestScanCapture()
  {
    state_ = State::CAPTURING;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent(
      "SCAN_CAPTURE_START",
      "station=" + currentStation().name + ", heading=" +
      std::to_string(heading_index_) + ".");
    capture_client_->async_send_request(
      captureRequest(currentStation().name, static_cast<uint8_t>(heading_index_)),
      [this, session, operation](rclcpp::Client<CaptureObjects>::SharedFuture future) {
        if (!callbackCurrent(session, operation) || state_ != State::CAPTURING) {
          return;
        }
        const auto response = future.get();
        if (!response->success) {
          emitEvent("SCAN_CAPTURE_FAILED", response->message +
            "; evidence=" + response->image_reference);
          handleCaptureFailure(response->message);
          return;
        }
        capture_retry_count_ = 0;
        ++successful_scan_captures_;
        for (const auto & observation : response->observations) {
          mergeObservation(observation);
        }
        emitEvent(
          "SCAN_CAPTURE_COMPLETE",
          response->message + "; evidence=" + response->image_reference + ".");
        finishScanHeading();
      });
  }

  void handleCaptureFailure(const std::string & reason)
  {
    if (reason.find("YOLO unavailable:") == 0U) {
      failMission("Perception model unavailable: " + reason);
      return;
    }
    if (capture_retry_count_ < config_.scan.capture_retries) {
      ++capture_retry_count_;
      emitEvent(
        "SCAN_CAPTURE_RETRY",
        reason + "; retry=" + std::to_string(capture_retry_count_) + ".");
      scheduleSettle(false);
      return;
    }
    emitEvent(
      "SCAN_HEADING_SKIPPED",
      reason + "; continuing after bounded capture retries.");
    capture_retry_count_ = 0;
    finishScanHeading();
  }

  void finishScanHeading()
  {
    scan_correction_count_ = 0;
    if (heading_index_ + 1 < config_.scan.turns) {
      ++heading_index_;
      sendScanSpin();
      return;
    }
    emitEvent(
      "STATION_SCAN_COMPLETE",
      "station=" + currentStation().name + ", confirmed_objects=" +
      std::to_string(station_observations_.size()) + ".");
    if (successful_scan_captures_ == 0) {
      failMission("Every heading capture failed; station was not inspected.");
      return;
    }
    writeRegistrySnapshot();
    requestPlanForStation();
  }

  bool allowedClass(const std::string & class_name) const
  {
    return std::find(
      config_.allowed_classes.begin(), config_.allowed_classes.end(),
      class_name) != config_.allowed_classes.end();
  }

  bool insideDropExclusion(const ObjectObservation & observation) const
  {
    return config_.isInDropZone(observation.centroid.x, observation.centroid.y);
  }

  void mergeObservation(const ObjectObservation & observation)
  {
    if (observation.object_uuid.empty() || !allowedClass(observation.class_name) ||
      observation.confidence < config_.min_confidence ||
      collected_objects_.count(observation.object_uuid) > 0U ||
      failed_objects_.count(observation.object_uuid) > 0U)
    {
      return;
    }
    if (insideDropExclusion(observation)) {
      emitEvent(
        "DROP_ZONE_OBJECT_IGNORED",
        "Ignoring UUID " + observation.object_uuid +
        " inside the collection exclusion radius.");
      return;
    }
    const auto current = station_observations_.find(observation.object_uuid);
    if (current == station_observations_.end() ||
      observation.confidence > current->second.confidence)
    {
      station_observations_[observation.object_uuid] = observation;
      known_objects_[observation.object_uuid] = observation;
    }
  }

  std::vector<ObjectObservation> pendingStationObservations() const
  {
    std::vector<ObjectObservation> values;
    for (const auto & entry : station_observations_) {
      if (collected_objects_.count(entry.first) == 0U &&
        failed_objects_.count(entry.first) == 0U &&
        !insideDropExclusion(entry.second))
      {
        values.push_back(entry.second);
      }
    }
    std::sort(
      values.begin(), values.end(),
      [](const ObjectObservation & left, const ObjectObservation & right) {
        return left.object_uuid < right.object_uuid;
      });
    return values;
  }

  void requestPlanForStation()
  {
    auto observations = pendingStationObservations();
    if (observations.empty()) {
      emitEvent("PLANNING_NOT_REQUIRED",
        "No eligible confirmed candidates; Gemini not called at " + currentStation().name);
      advanceStation();
      return;
    }
    // Finish the configured heading scan, then inspect each candidate without
    // perturbing scan headings. A failed inspection excludes only that UUID.
    for (const auto & observation : observations) {
      if (reviewed_objects_.count(observation.object_uuid) == 0U) {
        review_observation_ = observation;
        observation_turn_count_ = 0;
        alignObservation();
        return;
      }
    }
    if (observations.size() > 4U) {
      observations.resize(4U);  // Every candidate must have an attached image.
    }
    state_ = State::PLANNING;
    auto request = std::make_shared<PlanCleanup::Request>();
    request->mission_id = mission_id_;
    request->station_name = currentStation().name;
    request->drop_zone_name = config_.drop_zone_name;
    request->allowed_actions = {"collect_to_drop_zone", "skip"};
    request->observations = observations;

    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent(
      "PLANNING_START",
      "Sending " + std::to_string(observations.size()) +
      " validated candidates to the bounded planner.");
    planner_client_->async_send_request(
      request,
      [this, session, operation](rclcpp::Client<PlanCleanup>::SharedFuture future) {
        if (!callbackCurrent(session, operation) || state_ != State::PLANNING) {
          return;
        }
        const auto response = future.get();
        if (!response->success || response->action == "skip") {
          emitEvent(
            "PLANNING_SKIP",
            response->success ? response->reason : "Planner service failed.");
          advanceStation();
          return;
        }
        if (response->action != "collect_to_drop_zone") {
          failMission("Planner returned an action outside manager policy.");
          return;
        }
        const auto selected = station_observations_.find(response->object_uuid);
        if (selected == station_observations_.end()) {
          failMission("Planner returned an unknown object UUID.");
          return;
        }
        active_object_uuid_ = response->object_uuid;
        active_observation_ = selected->second;
        approach_attempts_ = 0;
        pick_retry_count_ = 0;
        emitEvent(
          "PLAN_ACCEPTED",
          "planner=" + response->planner_name + ", fallback=" +
          std::string(response->fallback_used ? "true" : "false") +
          ", UUID=" + active_object_uuid_ + ", reason=" + response->reason + ".");
        evaluateActiveGrasp(false);
      });
  }

  void alignObservation()
  {
    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!getRobotPose(x, y, yaw)) {
      finishObservation(false, "Robot pose unavailable");
      return;
    }
    const double dx = review_observation_.centroid.x - x;
    const double dy = review_observation_.centroid.y - y;
    const double distance = std::hypot(dx, dy);
    if (!std::isfinite(distance) || distance < config_.target_min_distance ||
      distance > config_.target_max_distance)
    {
      finishObservation(false, "Object outside observation distance limits");
      return;
    }
    const double error = std::atan2(std::sin(std::atan2(dy, dx) - yaw),
      std::cos(std::atan2(dy, dx) - yaw));
    if (std::abs(error) <= observation_yaw_tolerance_) {
      settleObservation(true);
      return;
    }
    if (observation_turn_count_++ >= observation_max_turns_) {
      finishObservation(false, "Observation heading did not converge");
      return;
    }
    state_ = State::OBSERVATION_ALIGNING;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent("OBSERVATION_ALIGN_START", "UUID=" + review_observation_.object_uuid +
      ", yaw_error=" + std::to_string(error));
    Spin::Goal goal;
    goal.target_yaw = static_cast<float>(std::clamp(
      error * spin_command_scale_, -3.141592653589793, 3.141592653589793));
    goal.time_allowance.sec = static_cast<int32_t>(scan_turn_timeout_);
    auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
    options.goal_response_callback =
      [this, session, operation](const GoalHandleSpin::SharedPtr & handle) {
        if (!callbackCurrent(session, operation)) {
          if (handle) {spin_client_->async_cancel_goal(handle);}
          return;
        }
        if (!handle) {
          finishObservation(false, "Nav2 rejected observation spin");
          return;
        }
        active_spin_goal_ = handle;
      };
    options.result_callback =
      [this, session, operation](const GoalHandleSpin::WrappedResult & result) {
        if (!callbackCurrent(session, operation)) {return;}
        active_spin_goal_.reset();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          finishObservation(false, "Observation spin failed");
          return;
        }
        settleObservation(false);
      };
    spin_client_->async_send_goal(goal, options);
  }

  void settleObservation(bool capture)
  {
    state_ = State::OBSERVATION_SETTLING;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    settle_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max<int64_t>(1,
        static_cast<int64_t>(config_.scan.settle_seconds * 1000.0))),
      [this, session, operation, capture]() {
        settle_timer_->cancel();
        if (!callbackCurrent(session, operation)) {return;}
        if (capture) {requestObservationCapture();} else {alignObservation();}
      });
  }

  void requestObservationCapture()
  {
    state_ = State::OBSERVATION_CAPTURING;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent("OBSERVATION_CAPTURE_START", "UUID=" + review_observation_.object_uuid);
    capture_client_->async_send_request(
      captureRequest(currentStation().name + "_observation", 249U),
      [this, session, operation](rclcpp::Client<CaptureObjects>::SharedFuture future) {
        if (!callbackCurrent(session, operation)) {return;}
        const auto response = future.get();
        if (response->success) {
          for (const auto & observation : response->observations) {
            if (observation.object_uuid == review_observation_.object_uuid &&
              allowedClass(observation.class_name) &&
              observation.confidence >= config_.min_confidence &&
              !insideDropExclusion(observation) && !observation.image_reference.empty())
            {
              station_observations_[observation.object_uuid] = observation;
              known_objects_[observation.object_uuid] = observation;
              finishObservation(true, response->message + "; evidence=" +
                observation.image_reference);
              return;
            }
          }
        }
        finishObservation(false, "Reserved UUID not confirmed: " + response->message +
          "; evidence=" + response->image_reference);
      });
  }

  void finishObservation(bool success, const std::string & reason)
  {
    const auto uuid = review_observation_.object_uuid;
    reviewed_objects_.insert(uuid);
    if (!success) {
      failed_objects_.insert(uuid);
      station_observations_.erase(uuid);
    }
    emitEvent(success ? "OBSERVATION_COMPLETE" : "OBSERVATION_SKIPPED",
      "UUID=" + uuid + "; " + reason);
    writeRegistrySnapshot();
    requestPlanForStation();
  }

  void evaluateActiveGrasp(bool after_reacquisition)
  {
    if (!active_observation_.grasp_valid) {
      if (!after_reacquisition) {
        requestTargetReacquisition();
      } else {
        recoverFromObjectFailure("No verified object-body centroid: " +
          active_observation_.grasp_reason);
      }
      return;
    }
    state_ = State::CHECKING_GRASP;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    auto request = std::make_shared<EvaluateGrasp::Request>();
    request->require_candidate = require_candidate_grasp_;
    request->target.header = active_observation_.header;
    request->target.point = active_observation_.centroid;
    emitEvent("GRASP_REACHABILITY_START", "UUID=" + active_object_uuid_ +
      "; attempt=" + std::to_string(approach_attempts_));
    grasp_check_client_->async_send_request(
      request, [this, session, operation, after_reacquisition](
        rclcpp::Client<EvaluateGrasp>::SharedFuture future) {
        if (!callbackCurrent(session, operation) || state_ != State::CHECKING_GRASP) {return;}
        const auto response = future.get();
        emitEvent("GRASP_REACHABILITY_RESULT", response->message);
        if (!response->success) {
          recoverFromObjectFailure("Grasp evaluation failed: " + response->message);
          return;
        }
        if (response->reachable) {
          if (after_reacquisition) {requestPick();} else {requestTargetReacquisition();}
          return;
        }
        if (!response->approach_available || approach_attempts_ >= max_approach_attempts_) {
          recoverFromObjectFailure("No reachable grasp after bounded approaches: " +
            response->message);
          return;
        }
        double x = 0.0, y = 0.0, yaw = 0.0;
        if (!getRobotPose(x, y, yaw)) {
          recoverFromObjectFailure("Robot pose unavailable for approach");
          return;
        }
        const auto & pose = response->approach_pose;
        const double distance = std::hypot(active_observation_.centroid.x - x,
          active_observation_.centroid.y - y);
        if (pose.header.frame_id != map_frame_ ||
          !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
          !std::isfinite(distance) || distance > config_.target_max_distance)
        {
          recoverFromObjectFailure("Invalid or out-of-range grasp approach pose");
          return;
        }
        ++approach_attempts_;
        emitEvent("OBJECT_REAPPROACH_START", "attempt=" + std::to_string(approach_attempts_) +
          "; goal=(" + std::to_string(pose.pose.position.x) + "," +
          std::to_string(pose.pose.position.y) + "); " + response->message);
        sendNavigationGoal(pose, NavigationPurpose::OBJECT_APPROACH);
      });
  }

  void requestTargetReacquisition(bool full_refresh = false)
  {
    state_ = State::REACQUIRING_TARGET;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent("TARGET_VIEW_START", "Looking down before fresh target capture; UUID=" +
      active_object_uuid_);
    observe_floor_client_->async_send_request(std::make_shared<Trigger::Request>(),
      [this, session, operation, full_refresh](rclcpp::Client<Trigger>::SharedFuture future) {
        if (!callbackCurrent(session, operation) || state_ != State::REACQUIRING_TARGET) {return;}
        if (!future.get()->success) {
          recoverFromObjectFailure("Floor observation pose failed");
          return;
        }
        settle_timer_ = this->create_wall_timer(350ms, [this, session, operation, full_refresh]() {
          settle_timer_->cancel();
          if (callbackCurrent(session, operation) && state_ == State::REACQUIRING_TARGET) {
            captureActiveTarget(full_refresh);
          }
        });
      });
  }

  void captureActiveTarget(bool full_refresh)
  {
    state_ = State::REACQUIRING_TARGET;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent(
      "TARGET_REACQUIRE_START",
      "Re-detecting reserved UUID " + active_object_uuid_ + " before pick.");
    capture_client_->async_send_request(
      captureRequest(currentStation().name + "_approach", full_refresh ? 249U : 250U),
      [this, session, operation](rclcpp::Client<CaptureObjects>::SharedFuture future) {
        if (!callbackCurrent(session, operation) ||
          state_ != State::REACQUIRING_TARGET)
        {
          return;
        }
        const auto response = future.get();
        const ObjectObservation * refreshed = response->success ?
          findActiveObservation(response->observations) : nullptr;
        if (!refreshed) {
          recoverFromObjectFailure("Reserved object was not reacquired before pick");
          return;
        }
        active_observation_ = *refreshed;
        known_objects_[active_object_uuid_] = active_observation_;
        evaluateActiveGrasp(true);
      });
  }

  const ObjectObservation * findActiveObservation(
    const std::vector<ObjectObservation> & observations) const
  {
    for (const auto & observation : observations) {
      if (observation.object_uuid == active_object_uuid_) {
        return &observation;
      }
    }
    return nullptr;
  }

  void requestPick()
  {
    if (require_candidate_grasp_ && active_observation_.grasp_strategy != "candidate_body") {
      recoverFromObjectFailure("Candidate body grasp was not validated: " +
        active_observation_.grasp_reason);
      return;
    }
    if (!active_observation_.grasp_valid) {
      recoverFromObjectFailure("Refusing grasp without full-mask central target");
      return;
    }
    state_ = State::PICKING;
    geometry_msgs::msg::PointStamped target;
    target.header = active_observation_.header;
    target.header.frame_id = map_frame_;
    const double age = (this->now() - rclcpp::Time(target.header.stamp)).seconds();
    if (age < 0.0 || age > 2.0) {
      recoverFromObjectFailure("Reacquired pick observation is stale");
      return;
    }
    target.point = active_observation_.centroid;
    pick_target_pub_->publish(target);

    const uint64_t session = mission_session_;
    const uint64_t timer_operation = ++operation_id_;
    emitEvent(
      "PICK_START",
      "Published fresh target for UUID " + active_object_uuid_ +
      "; strategy=" + active_observation_.grasp_strategy +
      "; " + active_observation_.grasp_reason);
    pick_timer_ = this->create_wall_timer(
      200ms,
      [this, session, timer_operation]() {
        pick_timer_->cancel();
        if (!callbackCurrent(session, timer_operation) || state_ != State::PICKING) {
          return;
        }
        const uint64_t service_operation = ++operation_id_;
        // Trigger cannot distinguish a pre-grasp failure from a failure after
        // closing the gripper. Recovery must allow for a partially held object.
        gripper_may_hold_object_ = true;
        pick_client_->async_send_request(
          std::make_shared<Trigger::Request>(),
          [this, session, service_operation](
            rclcpp::Client<Trigger>::SharedFuture future) {
            if (!callbackCurrent(session, service_operation) ||
              state_ != State::PICKING)
            {
              return;
            }
            const auto response = future.get();
            if (!response->success) {
              if (response->message.rfind("EMPTY_GRASP:", 0) == 0 &&
                pick_retry_count_ < max_pick_retries_)
              {
                ++pick_retry_count_;
                gripper_may_hold_object_ = false;
                emitEvent("PICK_RETRY", response->message);
                requestTargetReacquisition(true);
                return;
              }
              recoverFromObjectFailure("Pick failed: " + response->message);
              return;
            }
            gripper_may_hold_object_ = true;
            verifyPick();
          });
      });
  }

  void verifyPick()
  {
    state_ = State::VERIFYING_PICK;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent(
      "PICK_VERIFY_START",
      "Checking whether UUID " + active_object_uuid_ + " remains on the floor.");
    capture_client_->async_send_request(
      captureRequest(currentStation().name + "_pick_verify", 251U),
      [this, session, operation](rclcpp::Client<CaptureObjects>::SharedFuture future) {
        if (!callbackCurrent(session, operation) || state_ != State::VERIFYING_PICK) {
          return;
        }
        const auto response = future.get();
        const ObjectObservation * remaining = response->success ?
          findActiveObservation(response->observations) : nullptr;
        if (remaining) {
          if (pick_retry_count_ < max_pick_retries_) {
            ++pick_retry_count_;
            active_observation_ = *remaining;
            emitEvent(
              "PICK_RETRY",
              "Object still detected; retry=" +
              std::to_string(pick_retry_count_) + ".");
            // A failed pick invalidates old anchors. Look down and obtain a new
            // full-body observation; never reuse the clipped verification centroid.
            requestTargetReacquisition(true);
            return;
          }
          recoverFromObjectFailure("Object remained after all pick retries");
          return;
        }
        if (!response->success) {
          recoverFromObjectFailure("Pick verification unavailable; not claiming collection");
          return;
        } else {
          emitEvent("PICK_VERIFIED", "Reserved object is no longer at its floor pose.");
        }
        sendNavigationGoal(poseMessage(config_.drop_pose), NavigationPurpose::DROP_ZONE);
      });
  }

  void requestRelease()
  {
    state_ = State::RELEASING;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent(
      "RELEASE_START",
      "Lowering and placing the object at " + config_.drop_zone_name + ".");
    auto request = std::make_shared<PlaceObject::Request>();
    request->floor_target.header.frame_id = map_frame_;
    request->floor_target.header.stamp = now();
    request->floor_target.point.x = config_.placement_point.x;
    request->floor_target.point.y = config_.placement_point.y;
    request->floor_target.point.z = config_.placement_floor_height;
    request->release_height = config_.placement_release_height;
    place_client_->async_send_request(
      request,
      [this, session, operation](rclcpp::Client<PlaceObject>::SharedFuture future) {
        if (!callbackCurrent(session, operation) || state_ != State::RELEASING) {
          return;
        }
        const auto response = future.get();
        if (response->released) {gripper_may_hold_object_ = false;}
        if (!response->success || !response->released) {
          failMission("Object placement failed: " + response->message);
          return;
        }
        gripper_may_hold_object_ = false;
        requestPostDropPark();
      });
  }

  void requestPostDropPark()
  {
    state_ = State::PARKING_AFTER_DROP;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    park_arm_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this, session, operation](rclcpp::Client<Trigger>::SharedFuture future) {
        if (!callbackCurrent(session, operation) ||
          state_ != State::PARKING_AFTER_DROP)
        {
          return;
        }
        const auto response = future.get();
        if (!response->success) {
          failMission("Post-drop arm park failed: " + response->message);
          return;
        }
        collected_objects_.insert(active_object_uuid_);
        station_observations_.erase(active_object_uuid_);
        emitEvent(
          "OBJECT_COLLECTED",
          "UUID " + active_object_uuid_ + " released at " +
          config_.drop_zone_name + ".");
        writeRegistrySnapshot();
        navigateToCurrentStation(NavigationPurpose::RETURN_STATION);
      });
  }

  void recoverFromObjectFailure(const std::string & reason)
  {
    if (!active_object_uuid_.empty()) {
      failed_objects_.insert(active_object_uuid_);
      station_observations_.erase(active_object_uuid_);
    }
    emitEvent(
      "OBJECT_FAILED",
      reason + "; this object is skipped without aborting the station route.");
    if (gripper_may_hold_object_) {
      requestRecoveryRelease();
      return;
    }
    requestRecoveryPark();
  }

  void requestRecoveryRelease()
  {
    state_ = State::RECOVERING_RELEASE;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    emitEvent(
      "RECOVERY_RELEASE_START",
      "Opening the gripper because the failed pick may still be held.");
    open_gripper_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this, session, operation](rclcpp::Client<Trigger>::SharedFuture future) {
        if (!callbackCurrent(session, operation) ||
          state_ != State::RECOVERING_RELEASE)
        {
          return;
        }
        const auto response = future.get();
        if (!response->success) {
          failMission("Recovery release failed: " + response->message);
          return;
        }
        gripper_may_hold_object_ = false;
        requestRecoveryPark();
      });
  }

  void requestRecoveryPark()
  {
    state_ = State::RECOVERING_ARM;
    const uint64_t session = mission_session_;
    const uint64_t operation = ++operation_id_;
    park_arm_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this, session, operation](rclcpp::Client<Trigger>::SharedFuture future) {
        if (!callbackCurrent(session, operation) || state_ != State::RECOVERING_ARM) {
          return;
        }
        const auto response = future.get();
        if (!response->success) {
          failMission("Arm recovery park failed: " + response->message);
          return;
        }
        writeRegistrySnapshot();
        navigateToCurrentStation(NavigationPurpose::RETURN_STATION);
      });
  }

  void advanceStation()
  {
    station_observations_.clear();
    active_object_uuid_.clear();
    ++station_index_;
    if (station_index_ >= config_.stations.size()) {
      completeMission();
      return;
    }
    navigateToCurrentStation(NavigationPurpose::STATION);
  }

  void completeMission()
  {
    state_ = State::COMPLETE;
    emitEvent(
      "MISSION_COMPLETE",
      "collected=" + std::to_string(collected_objects_.size()) +
      ", failed=" + std::to_string(failed_objects_.size()) +
      ", incomplete_scans=" + std::to_string(incomplete_scan_count_) + ".");
    writeRegistrySnapshot();
    state_ = State::IDLE;
    navigation_purpose_ = NavigationPurpose::NONE;
    publishStatus();
  }

  const TaskStation & currentStation() const
  {
    return config_.stations.at(station_index_);
  }

  bool getRobotPose(double & x, double & y, double & yaw, const std::string & frame = "")
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        frame.empty() ? map_frame_ : frame, "base_link", tf2::TimePointZero,
        tf2::durationFromSec(0.1));
      x = transform.transform.translation.x;
      y = transform.transform.translation.y;
      yaw = tf2::getYaw(transform.transform.rotation);
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Robot pose unavailable: %s", exception.what());
      return false;
    }
  }

  void cancelActiveOperations()
  {
    ++mission_session_;
    ++operation_id_;
    if (navigation_retry_timer_) {navigation_retry_timer_->cancel();}
    if (settle_timer_) {
      settle_timer_->cancel();
    }
    if (pick_timer_) {
      pick_timer_->cancel();
    }
    if (active_navigation_goal_) {
      nav_client_->async_cancel_goal(active_navigation_goal_);
      active_navigation_goal_.reset();
    }
    if (active_spin_goal_) {
      spin_client_->async_cancel_goal(active_spin_goal_);
      active_spin_goal_.reset();
    }
  }

  void failMission(const std::string & reason)
  {
    cancelActiveOperations();
    state_ = State::IDLE;
    navigation_purpose_ = NavigationPurpose::NONE;
    emitEvent("MISSION_FAILED", reason);
    writeRegistrySnapshot();
  }

  static std::string jsonEscape(const std::string & input)
  {
    std::ostringstream output;
    for (const char character : input) {
      if (character == '"' || character == '\\') {
        output << '\\';
      }
      if (character == '\n') {
        output << "\\n";
      } else {
        output << character;
      }
    }
    return output.str();
  }

  void startEvidenceSession()
  {
    if (mission_log_.is_open()) {
      mission_log_.close();
    }
    const std::time_t now = std::time(nullptr);
    std::tm local_time{};
    localtime_r(&now, &local_time);
    std::ostringstream name;
    name << "run_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << "_" <<
      static_cast<long>(::getpid());
    mission_id_ = name.str();
    mission_directory_ =
      std::filesystem::path(debug_output_root_) / mission_id_;
    std::filesystem::create_directories(mission_directory_);
    mission_log_.open(mission_directory_ / "mission.log", std::ios::app);
  }

  void writeRegistrySnapshot() const
  {
    if (mission_directory_.empty()) {
      return;
    }
    std::ofstream output(mission_directory_ / "object_registry.json");
    output << "{\n  \"mission_id\": \"" << jsonEscape(mission_id_) << "\",\n";
    output << "  \"collected_count\": " << collected_objects_.size() << ",\n";
    output << "  \"failed_count\": " << failed_objects_.size() << ",\n";
    output << "  \"incomplete_scan_count\": " << incomplete_scan_count_ << ",\n";
    output << "  \"objects\": [\n";
    bool first = true;
    for (const auto & entry : known_objects_) {
      if (!first) {
        output << ",\n";
      }
      first = false;
      const std::string state = collected_objects_.count(entry.first) > 0U ?
        "COLLECTED" : (failed_objects_.count(entry.first) > 0U ? "FAILED" : "CONFIRMED");
      output << "    {\"uuid\": \"" << jsonEscape(entry.first) <<
        "\", \"class\": \"" << jsonEscape(entry.second.class_name) <<
        "\", \"state\": \"" << state << "\", \"x\": " <<
        entry.second.centroid.x << ", \"y\": " << entry.second.centroid.y <<
        ", \"z\": " << entry.second.centroid.z << "}";
    }
    output << "\n  ]\n}\n";
  }

  void emitEvent(const std::string & tag, const std::string & details)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] %s", tag.c_str(), details.c_str());
    if (mission_log_.is_open()) {
      mission_log_ << std::fixed << std::setprecision(6) <<
        this->now().seconds() << " [" << tag << "] " <<
        details << std::endl;
    }
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
    value << "state=" << stateName() << ";mission=" <<
      (mission_id_.empty() ? "none" : mission_id_) << ";station=";
    if (station_index_ < config_.stations.size()) {
      value << currentStation().name;
    } else {
      value << "none";
    }
    value << ";station_index=" << station_index_ << ";station_count=" <<
      config_.stations.size() << ";heading=" << heading_index_ << "/" <<
      config_.scan.turns << ";active_object=" <<
      (active_object_uuid_.empty() ? "none" : active_object_uuid_) <<
      ";collected=" << collected_objects_.size() << ";failed=" <<
      failed_objects_.size() << ";localization=" <<
      (localization_mode_.empty() ? "UNKNOWN" : localization_mode_);
    status.data = value.str();
    status_pub_->publish(status);
  }

  std::string task_config_path_;
  std::unique_ptr<NavigationEvidence> navigation_evidence_;
  geometry_msgs::msg::PoseStamped last_navigation_pose_;
  rclcpp::TimerBase::SharedPtr navigation_retry_timer_;
  int max_navigation_retries_{2}, navigation_retry_count_{0};
  double navigation_retry_delay_{2.0}, navigation_retry_timeout_{10.0};
  std::chrono::steady_clock::time_point guard_received_{}, localization_received_{};
  std::string map_frame_;
  std::string navigate_action_;
  std::string spin_action_;
  std::string debug_output_root_;
  std::string config_error_;
  std::string localization_mode_;
  std::string mission_id_;
  double spin_command_scale_{1.0};
  double scan_turn_timeout_{30.0};
  int max_pick_retries_{1};
  int max_approach_attempts_{3};
  int approach_attempts_{0};
  std::string approach_behavior_tree_;
  double scan_start_yaw_{0.0};
  int scan_correction_count_{0};
  double observation_yaw_tolerance_{0.10};
  int observation_max_turns_{3};
  int observation_turn_count_{0};
  int successful_scan_captures_{0};
  int incomplete_scan_count_{0};
  ObjectObservation review_observation_;
  std::unordered_set<std::string> reviewed_objects_;
  TaskConfig config_;
  bool config_loaded_{false};
  bool gripper_may_hold_object_{false};
  State state_{State::IDLE};
  NavigationPurpose navigation_purpose_{NavigationPurpose::NONE};
  uint64_t mission_session_{0U};
  uint64_t operation_id_{0U};
  size_t station_index_{0U};
  int heading_index_{0};
  bool require_candidate_grasp_{false};
  int capture_retry_count_{0};
  int pick_retry_count_{0};
  std::string active_object_uuid_;
  ObjectObservation active_observation_;
  std::unordered_map<std::string, ObjectObservation> station_observations_;
  std::unordered_map<std::string, ObjectObservation> known_objects_;
  std::unordered_set<std::string> collected_objects_;
  std::unordered_set<std::string> failed_objects_;
  std::filesystem::path mission_directory_;
  std::ofstream mission_log_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNavigate::SharedPtr active_navigation_goal_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  GoalHandleSpin::SharedPtr active_spin_goal_;
  rclcpp::Client<CaptureObjects>::SharedPtr capture_client_;
  rclcpp::Client<PlanCleanup>::SharedPtr planner_client_;
    rclcpp::Client<Trigger>::SharedPtr pick_client_;
  rclcpp::Client<EvaluateGrasp>::SharedPtr grasp_check_client_;
  rclcpp::Client<Trigger>::SharedPtr open_gripper_client_;
  rclcpp::Client<PlaceObject>::SharedPtr place_client_;
    rclcpp::Client<Trigger>::SharedPtr park_arm_client_;
    rclcpp::Client<Trigger>::SharedPtr observe_floor_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr motion_guard_sub_;
  std::string motion_guard_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pick_target_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr settle_timer_;
  rclcpp::TimerBase::SharedPtr pick_timer_;
};

}  // namespace cleanup_task_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cleanup_task_manager::CleanupTaskManager>());
  rclcpp::shutdown();
  return 0;
}
