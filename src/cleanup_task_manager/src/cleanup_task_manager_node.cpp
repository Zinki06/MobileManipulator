#include "cleanup_task_manager/task_config.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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

using namespace std::chrono_literals;

namespace cleanup_task_manager
{

class CleanupTaskManager : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Trigger = std_srvs::srv::Trigger;

  CleanupTaskManager()
  : Node("cleanup_task_manager")
  {
    this->declare_parameter<std::string>("task_config_path", "");
    this->declare_parameter<std::string>("map_frame", "map");
    task_config_path_ = this->get_parameter("task_config_path").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    loadConfig();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "/navigate_to_pose");

    start_patrol_client_ = this->create_client<Trigger>("/start_marker_patrol");
    stop_patrol_client_ = this->create_client<Trigger>("/stop_marker_patrol");
    pick_client_ = this->create_client<Trigger>("/execute_pick_and_place");
    open_gripper_client_ = this->create_client<Trigger>("/open_gripper");
    park_arm_client_ = this->create_client<Trigger>("/park_arm");

    object_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/object_centroid", 10,
      std::bind(&CleanupTaskManager::objectCallback, this, std::placeholders::_1));
    route_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/route_navigation/status", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(
        &CleanupTaskManager::routeStatusCallback, this, std::placeholders::_1));
    localization_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/aruco/localization_mode", 10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        localization_mode_ = message->data;
      });

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/cleanup/status", rclcpp::QoS(1).transient_local().reliable());
    event_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/cleanup/events", 20);
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
    emitEvent("NODE_INIT", "Cleanup task manager is idle.");
  }

private:
  enum class State
  {
    IDLE,
    STARTING_PATROL,
    PATROLLING,
    STOPPING_PATROL,
    NAVIGATING_TO_OBJECT,
    PICKING,
    NAVIGATING_TO_DROP,
    RELEASING,
    PARKING
  };

  enum class NavigationPurpose
  {
    NONE,
    OBJECT_APPROACH,
    DROP_ZONE
  };

  void loadConfig()
  {
    if (task_config_path_.empty()) {
      config_error_ = "task_config_path parameter is empty";
      return;
    }
    try {
      config_ = TaskConfig::loadFromFile(task_config_path_);
      config_loaded_ = true;
    } catch (const std::exception & exception) {
      config_error_ = exception.what();
      RCLCPP_ERROR(this->get_logger(), "Task config error: %s", exception.what());
    }
  }

  std::string stateName() const
  {
    switch (state_) {
      case State::IDLE:
        return "IDLE";
      case State::STARTING_PATROL:
        return "STARTING_PATROL";
      case State::PATROLLING:
        return "PATROLLING";
      case State::STOPPING_PATROL:
        return "STOPPING_PATROL";
      case State::NAVIGATING_TO_OBJECT:
        return "NAVIGATING_TO_OBJECT";
      case State::PICKING:
        return "PICKING";
      case State::NAVIGATING_TO_DROP:
        return "NAVIGATING_TO_DROP";
      case State::RELEASING:
        return "RELEASING";
      case State::PARKING:
        return "PARKING";
    }
    return "UNKNOWN";
  }

  bool serviceReady(const rclcpp::Client<Trigger>::SharedPtr & client) const
  {
    return client->service_is_ready();
  }

  void handleStart(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (state_ != State::IDLE) {
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
      response->message =
        "Set cleanup.drop_pose.configured=true after teaching a safe drop pose.";
      return;
    }
    if (localization_mode_.empty() || localization_mode_ == "UNINITIALIZED" ||
      localization_mode_ == "DEGRADED")
    {
      response->success = false;
      response->message = "Cleanup requires initialized, non-degraded localization.";
      return;
    }
    if (!serviceReady(start_patrol_client_) || !serviceReady(stop_patrol_client_) ||
      !serviceReady(pick_client_) || !serviceReady(open_gripper_client_) ||
      !serviceReady(park_arm_client_) || !nav_client_->action_server_is_ready())
    {
      response->success = false;
      response->message = "Cleanup dependencies are not ready.";
      return;
    }

    requestPatrolStart();
    response->success = true;
    response->message = "Cleanup patrol start requested.";
  }

  void handleStop(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (state_ == State::IDLE) {
      response->success = false;
      response->message = "Cleanup is not active.";
      return;
    }

    ++navigation_goal_id_;
    if (active_navigation_goal_) {
      nav_client_->async_cancel_goal(active_navigation_goal_);
      active_navigation_goal_.reset();
    }
    if (serviceReady(stop_patrol_client_)) {
      stop_patrol_client_->async_send_request(std::make_shared<Trigger::Request>());
    }
    state_ = State::IDLE;
    navigation_purpose_ = NavigationPurpose::NONE;
    emitEvent(
      "CLEANUP_STOPPED",
      "Cleanup stopped; an already-running arm service cannot be preempted.");
    response->success = true;
    response->message = "Cleanup stop requested.";
  }

  void requestPatrolStart()
  {
    state_ = State::STARTING_PATROL;
    saw_route_busy_ = false;
    emitEvent("PATROL_START_REQUEST", "Requesting the configured virtual patrol.");
    start_patrol_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture future) {
        if (state_ != State::STARTING_PATROL) {
          return;
        }
        const auto response = future.get();
        if (!response->success) {
          failMission("Patrol start failed: " + response->message);
          return;
        }
        state_ = State::PATROLLING;
        emitEvent("PATROL_STARTED", response->message);
      });
  }

  void routeStatusCallback(const std_msgs::msg::String::SharedPtr message)
  {
    route_is_idle_ = message->data.rfind("state=IDLE", 0U) == 0U;
    if (!route_is_idle_) {
      saw_route_busy_ = true;
    }

    if (state_ == State::STOPPING_PATROL && route_is_idle_) {
      sendNavigationGoal(approach_pose_, NavigationPurpose::OBJECT_APPROACH);
      return;
    }

    // /start_marker_patrol currently performs one outward and one return leg.
    // Restart only after observing that this patrol actually became busy.
    if (state_ == State::PATROLLING && route_is_idle_ && saw_route_busy_) {
      requestPatrolStart();
    }
  }

  void objectCallback(const geometry_msgs::msg::PointStamped::SharedPtr message)
  {
    if (state_ != State::PATROLLING) {
      return;
    }
    if (last_cycle_completion_.nanoseconds() != 0 &&
      (this->now() - last_cycle_completion_).seconds() < config_.target_cooldown)
    {
      return;
    }
    if (message->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring object target without a frame_id");
      return;
    }

    if (message->header.stamp.sec != 0 || message->header.stamp.nanosec != 0U) {
      const rclcpp::Time target_stamp(message->header.stamp);
      const double age = (this->now() - target_stamp).seconds();
      if (age < 0.0 || age > config_.target_max_age) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Ignoring stale object target (age %.2fs)", age);
        return;
      }
    }

    geometry_msgs::msg::PointStamped target_in_map;
    try {
      target_in_map = tf_buffer_->transform(
        *message, map_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Ignoring target with unavailable TF: %s", exception.what());
      return;
    }

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (!getRobotPose(robot_x, robot_y, robot_yaw)) {
      return;
    }
    (void)robot_yaw;

    const double dx = target_in_map.point.x - robot_x;
    const double dy = target_in_map.point.y - robot_y;
    const double distance = std::hypot(dx, dy);
    if (distance < config_.target_min_distance ||
      distance > config_.target_max_distance)
    {
      return;
    }

    const double heading = std::atan2(dy, dx);
    approach_pose_.header.frame_id = map_frame_;
    approach_pose_.pose.position.x =
      target_in_map.point.x - config_.approach_standoff * std::cos(heading);
    approach_pose_.pose.position.y =
      target_in_map.point.y - config_.approach_standoff * std::sin(heading);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, heading);
    approach_pose_.pose.orientation = tf2::toMsg(orientation);
    active_target_ = target_in_map;

    state_ = State::STOPPING_PATROL;
    saw_route_busy_ = false;
    std::ostringstream details;
    details << "Target accepted at map=(" << target_in_map.point.x << ", " <<
      target_in_map.point.y << "); stopping patrol before approach.";
    emitEvent("TARGET_ACCEPTED", details.str());

    stop_patrol_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture future) {
        if (state_ != State::STOPPING_PATROL) {
          return;
        }
        const auto response = future.get();
        if (!response->success && route_is_idle_) {
          sendNavigationGoal(approach_pose_, NavigationPurpose::OBJECT_APPROACH);
        }
      });
  }

  void sendNavigationGoal(
    geometry_msgs::msg::PoseStamped pose, NavigationPurpose purpose)
  {
    pose.header.stamp = this->now();
    NavigateToPose::Goal goal;
    goal.pose = pose;
    navigation_purpose_ = purpose;
    state_ = purpose == NavigationPurpose::OBJECT_APPROACH ?
      State::NAVIGATING_TO_OBJECT : State::NAVIGATING_TO_DROP;
    const uint64_t goal_id = ++navigation_goal_id_;
    emitEvent(
      purpose == NavigationPurpose::OBJECT_APPROACH ?
      "OBJECT_APPROACH_START" : "DROP_NAVIGATION_START",
      "Sending mission-level Nav2 goal_id=" + std::to_string(goal_id));

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this, goal_id](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
        if (goal_id != navigation_goal_id_ || state_ == State::IDLE) {
          if (goal_handle) {
            nav_client_->async_cancel_goal(goal_handle);
          }
          return;
        }
        if (!goal_handle) {
          failMission("Nav2 rejected a cleanup mission goal.");
          return;
        }
        active_navigation_goal_ = goal_handle;
      };
    options.result_callback =
      [this, goal_id](const GoalHandleNavigateToPose::WrappedResult & result) {
        navigationResult(result, goal_id);
      };
    nav_client_->async_send_goal(goal, options);
  }

  void navigationResult(
    const GoalHandleNavigateToPose::WrappedResult & result, uint64_t goal_id)
  {
    if (goal_id != navigation_goal_id_ || state_ == State::IDLE) {
      return;
    }
    active_navigation_goal_.reset();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      failMission("Cleanup mission Nav2 goal did not succeed.");
      return;
    }

    if (navigation_purpose_ == NavigationPurpose::OBJECT_APPROACH) {
      requestPick();
    } else if (navigation_purpose_ == NavigationPurpose::DROP_ZONE) {
      requestRelease();
    }
  }

  void requestPick()
  {
    state_ = State::PICKING;
    active_target_.header.stamp = this->now();
    pick_target_pub_->publish(active_target_);
    emitEvent("PICK_START", "Published the selected target for the pick sequence.");

    // Topic and service delivery are independent; allow the pick node to receive
    // the selected target before triggering its service.
    pick_request_timer_ = this->create_wall_timer(
      200ms,
      [this]() {
        pick_request_timer_->cancel();
        if (state_ != State::PICKING) {
          return;
        }
        pick_client_->async_send_request(
          std::make_shared<Trigger::Request>(),
          [this](rclcpp::Client<Trigger>::SharedFuture future) {
            if (state_ != State::PICKING) {
              return;
            }
            const auto response = future.get();
            if (!response->success) {
              failMission("Pick failed: " + response->message);
              return;
            }
            geometry_msgs::msg::PoseStamped drop_pose;
            drop_pose.header.frame_id = map_frame_;
            drop_pose.pose.position.x = config_.drop_pose.x;
            drop_pose.pose.position.y = config_.drop_pose.y;
            tf2::Quaternion orientation;
            orientation.setRPY(0.0, 0.0, config_.drop_pose.yaw);
            drop_pose.pose.orientation = tf2::toMsg(orientation);
            sendNavigationGoal(drop_pose, NavigationPurpose::DROP_ZONE);
          });
      });
  }

  void requestRelease()
  {
    state_ = State::RELEASING;
    emitEvent("RELEASE_START", "Opening the gripper at the configured drop pose.");
    open_gripper_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture future) {
        if (state_ != State::RELEASING) {
          return;
        }
        const auto response = future.get();
        if (!response->success) {
          failMission("Object release failed: " + response->message);
          return;
        }
        requestParkArm();
      });
  }

  void requestParkArm()
  {
    state_ = State::PARKING;
    park_arm_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this](rclcpp::Client<Trigger>::SharedFuture future) {
        if (state_ != State::PARKING) {
          return;
        }
        const auto response = future.get();
        if (!response->success) {
          failMission("Arm park failed: " + response->message);
          return;
        }
        last_cycle_completion_ = this->now();
        emitEvent("CLEANUP_CYCLE_COMPLETE", "Object released; resuming patrol.");
        requestPatrolStart();
      });
  }

  bool getRobotPose(double & x, double & y, double & yaw)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_, "base_link", tf2::TimePointZero,
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

  void failMission(const std::string & reason)
  {
    state_ = State::IDLE;
    navigation_purpose_ = NavigationPurpose::NONE;
    emitEvent("CLEANUP_FAILED", reason);
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
    status.data = "state=" + stateName() + ";localization=" +
      (localization_mode_.empty() ? "UNKNOWN" : localization_mode_);
    status_pub_->publish(status);
  }

  std::string task_config_path_;
  std::string map_frame_;
  std::string config_error_;
  std::string localization_mode_;
  TaskConfig config_;
  bool config_loaded_{false};
  bool route_is_idle_{true};
  bool saw_route_busy_{false};
  State state_{State::IDLE};
  NavigationPurpose navigation_purpose_{NavigationPurpose::NONE};
  uint64_t navigation_goal_id_{0U};
  rclcpp::Time last_cycle_completion_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::PointStamped active_target_;
  geometry_msgs::msg::PoseStamped approach_pose_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNavigateToPose::SharedPtr active_navigation_goal_;
  rclcpp::Client<Trigger>::SharedPtr start_patrol_client_;
  rclcpp::Client<Trigger>::SharedPtr stop_patrol_client_;
  rclcpp::Client<Trigger>::SharedPtr pick_client_;
  rclcpp::Client<Trigger>::SharedPtr open_gripper_client_;
  rclcpp::Client<Trigger>::SharedPtr park_arm_client_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr route_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_mode_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pick_target_pub_;
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr pick_request_timer_;
};

}  // namespace cleanup_task_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cleanup_task_manager::CleanupTaskManager>());
  rclcpp::shutdown();
  return 0;
}
