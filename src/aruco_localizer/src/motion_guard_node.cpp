#include <algorithm>
#include "aruco_localizer/localization_policy.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Single final velocity writer. Navigation and recovery must never bypass it.
class MotionGuard : public rclcpp::Node
{
public:
  MotionGuard() : Node("motion_guard")
  {
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    output_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_ = create_publisher<std_msgs::msg::String>(
      "/motion_guard/status", rclcpp::QoS(1).transient_local());
    input_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_collision_checked", 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        command_ = *msg;
        command_time_ = std::chrono::steady_clock::now();
        command_received_ = true;
      });
    intent_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_smoothed", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        intent_ = *msg;
        intent_time_ = std::chrono::steady_clock::now();
      });
    inhibit_sub_ = create_subscription<std_msgs::msg::String>(
      "/motion/inhibit", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (!msg->data.empty()) {fault_ = "motion executor: " + msg->data;}
      });
    mode_ = create_subscription<std_msgs::msg::String>(
      "/aruco/localization_mode", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        mode_value_ = msg->data;
        mode_time_ = std::chrono::steady_clock::now();
        if (mode_value_ == "DEGRADED") {fault_ = "localization degraded";}
      });
    scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        const bool valid = std::any_of(msg->ranges.begin(), msg->ranges.end(),
          [msg](float range) {
            return (std::isfinite(range) && range >= msg->range_min &&
              range <= msg->range_max) || (std::isinf(range) && range > 0.0);
          });
        if (valid) {scan_header_ = msg->header;}
      });
    depth_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cleanup/obstacle_points", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (msg->width * msg->height >= 20) {depth_header_ = msg->header;}
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {tick();});
  }

private:
  std::string sourceIssue(const std_msgs::msg::Header & header, std::string & detail)
  {
    if (header.frame_id.empty()) {detail = "no valid message received"; return "missing";}
    const auto stamp = rclcpp::Time(header.stamp);
    const double age = (get_clock()->now() - stamp).seconds();
    detail = "frame=" + header.frame_id + " age=" + std::to_string(age) + "s";
    if (age < -0.1) {return "future timestamp";}
    if (age >= 0.5) {return "stale";}
    std::string error;
    if (!buffer_->canTransform("base_link", header.frame_id, stamp,
      rclcpp::Duration::from_seconds(0.0), &error))
    {
      detail += " TF=" + error;
      return "TF unavailable";
    }
    return "";
  }

  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    std::string blocked = fault_;
    if (blocked.empty() && (mode_value_.empty() || mode_value_ == "UNINITIALIZED" ||
      std::chrono::duration<double>(now - mode_time_).count() > 0.5))
    {
      blocked = "waiting for fresh localization";
    }
    try {
      const auto tf = buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
      const auto & p = tf.transform.translation;
      const double yaw = tf2::getYaw(tf.transform.rotation);
      const double age = (get_clock()->now() - rclcpp::Time(tf.header.stamp)).seconds();
      if (age < -0.1 || age > 0.3) {blocked = "stale map transform";}
      const auto odom = buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
      const double odom_age = (get_clock()->now() - rclcpp::Time(odom.header.stamp)).seconds();
      if (odom_age < -0.1 || odom_age > 0.3) {blocked = "stale odom transform";}
      const auto & robot = odom.transform.translation;
      const double displacement = aruco_localizer::correctionDisplacement(
        {x_, y_, yaw_}, {p.x, p.y, yaw}, robot.x, robot.y);
      if (!std::isfinite(displacement) || !std::isfinite(yaw)) {
        fault_ = "non-finite localization transform";
      }
      if (have_transform_ && (displacement > 0.12 ||
        std::abs(std::atan2(std::sin(yaw - yaw_), std::cos(yaw - yaw_))) > 0.20))
      {
        fault_ = "map transform discontinuity; restart only after localization inspection";
      }
      x_ = p.x;
      y_ = p.y;
      yaw_ = yaw;
      have_transform_ = true;
    } catch (const tf2::TransformException &) {
      blocked = "map transform unavailable";
    }
    if (!fault_.empty()) {blocked = fault_;}
    if (blocked.empty()) {
      std::string laser_detail, depth_detail;
      const auto laser_issue = sourceIssue(scan_header_, laser_detail);
      const auto depth_issue = sourceIssue(depth_header_, depth_detail);
      if (!laser_issue.empty() || !depth_issue.empty()) {
        blocked = !laser_issue.empty() ? "laser " + laser_issue : "depth " + depth_issue;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[MOTION_SENSOR] laser={%s; %s} depth={%s; %s}; limit=0.500s",
          laser_issue.empty() ? "OK" : laser_issue.c_str(), laser_detail.c_str(),
          depth_issue.empty() ? "OK" : depth_issue.c_str(), depth_detail.c_str());
      }
    }
    if (count_publishers("/cmd_vel") > 1) {
      fault_ = "multiple final velocity publishers";
      blocked = fault_;
    }
    if (!command_received_ ||
      std::chrono::duration<double>(now - command_time_).count() > 0.25)
    {
      if (blocked.empty()) {blocked = "command timeout";}
    }
    if (!std::isfinite(command_.linear.x) || !std::isfinite(command_.angular.z)) {
      blocked = "invalid command";
    }
    if (blocked.empty() &&
      std::chrono::duration<double>(now - intent_time_).count() < 0.25 &&
      (std::abs(intent_.linear.x) > 0.001 || std::abs(intent_.angular.z) > 0.001) &&
      std::abs(command_.linear.x) < 0.0001 && std::abs(command_.angular.z) < 0.0001)
    {
      blocked = "collision monitor stopped motion";
    }
    geometry_msgs::msg::Twist next;
    if (blocked.empty()) {
      // Limit acceleration, but never delay collision-monitor braking to zero.
      const double desired = std::clamp(command_.linear.x, -0.05, 0.10);
      const double angular = std::clamp(command_.angular.z, -0.35, 0.35);
      next.linear.x = std::abs(desired) < std::abs(previous_.linear.x) ? desired :
        std::clamp(desired, previous_.linear.x - 0.003, previous_.linear.x + 0.003);
      next.angular.z = std::abs(angular) < std::abs(previous_.angular.z) ? angular :
        std::clamp(angular, previous_.angular.z - 0.008, previous_.angular.z + 0.008);
      if (desired * previous_.linear.x < 0.0) {next.linear.x = 0.0;}
      if (angular * previous_.angular.z < 0.0) {next.angular.z = 0.0;}
    }
    output_->publish(next);
    if (std::abs(next.linear.x) > 0.001 || std::abs(next.angular.z) > 0.001) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[MOTION_COMMAND] requested=(%.3f,%.3f) output=(%.3f,%.3f)",
        command_.linear.x, command_.angular.z, next.linear.x, next.angular.z);
    }
    previous_ = next;
    std_msgs::msg::String status;
    status.data = fault_.empty() ? (blocked.empty() ? "READY" : "BLOCKED: " + blocked) :
      "FAULT: " + fault_;
    status_->publish(status);
    if (status.data != last_status_) {
      RCLCPP_INFO(get_logger(), "[MOTION_GUARD] %s", status.data.c_str());
      last_status_ = status.data;
    }
  }

  using Steady = std::chrono::steady_clock;
  Steady::time_point command_time_{Steady::now()}, mode_time_{Steady::now()};
  bool command_received_{false}, have_transform_{false};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  std::string fault_, mode_value_, last_status_;
  geometry_msgs::msg::Twist command_, previous_;
  geometry_msgs::msg::Twist intent_;
  Steady::time_point intent_time_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr intent_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr inhibit_sub_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_;
  std_msgs::msg::Header scan_header_, depth_header_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionGuard>());
  rclcpp::shutdown();
  return 0;
}
