#include "aruco_localizer/route_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
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
#include <vector>

using namespace std::chrono_literals;

namespace aruco_localizer
{

class ArucoWaypointNavigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
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

    event_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/route_navigation/events", 50);
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/route_navigation/status", rclcpp::QoS(1).transient_local().reliable());
    path_visual_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/route_navigation/path", rclcpp::QoS(1).transient_local().reliable());

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
  enum class ExecutionState
  {
    IDLE,
    SENDING_GOAL,
    NAVIGATING,
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
    this->declare_parameter<int>("max_navigation_retries", 2);
    this->declare_parameter<int>("retry_cooldown_ms", 500);
    this->declare_parameter<double>("recovery_arrival_tolerance", 0.15);

    route_yaml_path_ = this->get_parameter("route_yaml_path").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    navigate_to_pose_action_ =
      this->get_parameter("navigate_to_pose_action").as_string();
    max_navigation_retries_ =
      this->get_parameter("max_navigation_retries").as_int();
    retry_cooldown_ms_ = this->get_parameter("retry_cooldown_ms").as_int();
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
    std::string & message)
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

    active_route_name_ = route_name;
    active_waypoint_names_ = route_config_.route(route_name);
    pending_patrol_route_ = patrol_return;
    waypoint_index_ = 0U;
    retry_count_ = 0;
    state_ = ExecutionState::SENDING_GOAL;

    emitEvent("ROUTE_START", "Starting virtual route '" + route_name + "'.");
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
      ++waypoint_index_;
      retry_count_ = 0;
      sendCurrentGoal();
      return;
    }

    if (result.code == rclcpp_action::ResultCode::ABORTED &&
      robotIsWithinRecoveryTolerance())
    {
      emitEvent(
        "WAYPOINT_RECOVERY_REACHED",
        "Nav2 aborted after reaching the configured recovery tolerance for '" +
        currentWaypointName() + "'.");
      ++waypoint_index_;
      retry_count_ = 0;
      sendCurrentGoal();
      return;
    }

    const std::string result_name =
      result.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" : "ABORTED";
    handleGoalFailure("Nav2 result=" + result_name, goal_id);
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
    waypoint_index_ = 0U;
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
    if (state_ == ExecutionState::RETRY_WAIT) {
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
    // When a goal is still being sent, goalResponseCallback cancels it as soon
    // as Nav2 returns the handle. IDLE is published only after its final result.
  }

  void finishAbort(const std::string & reason)
  {
    const std::string route_name = active_route_name_;
    ++current_goal_id_;
    active_goal_handle_.reset();

    state_ = ExecutionState::IDLE;
    active_route_name_.clear();
    active_waypoint_names_.clear();
    pending_patrol_route_.clear();
    pending_abort_reason_.clear();
    waypoint_index_ = 0U;
    retry_count_ = 0;
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
      ";count=" << active_waypoint_names_.size() << ";retry=" << retry_count_;
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
  int max_navigation_retries_{2};
  int retry_cooldown_ms_{500};
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
  uint64_t current_goal_id_{0U};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  GoalHandleNavigateToPose::SharedPtr active_goal_handle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_visual_pub_;
  rclcpp::Service<Trigger>::SharedPtr nav_to_0_service_;
  rclcpp::Service<Trigger>::SharedPtr nav_to_5_service_;
  rclcpp::Service<Trigger>::SharedPtr start_patrol_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_patrol_service_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
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
