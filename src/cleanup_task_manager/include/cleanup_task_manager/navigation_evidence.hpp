#pragma once

#include <filesystem>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cleanup_task_manager
{
// Read-only diagnostics: no costmap clearing, TF changes, or velocity publishing.
class NavigationEvidence
{
public:
  explicit NavigationEvidence(rclcpp::Node & node);
  bool stationary() const;
  void save(
    const std::filesystem::path & directory,
    const geometry_msgs::msg::PoseStamped & goal,
    const geometry_msgs::msg::PoseStamped & robot,
    const std::string & reason, const std::string & guard) const;

private:
  rclcpp::Node & node_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr global_, local_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_sub_, local_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
}  // namespace cleanup_task_manager
