#include "cleanup_task_manager/navigation_evidence.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace cleanup_task_manager
{
namespace
{
YAML::Node poseNode(const geometry_msgs::msg::Pose & pose)
{
  YAML::Node node;
  node["position"].push_back(pose.position.x);
  node["position"].push_back(pose.position.y);
  node["position"].push_back(pose.position.z);
  node["quaternion_xyzw"].push_back(pose.orientation.x);
  node["quaternion_xyzw"].push_back(pose.orientation.y);
  node["quaternion_xyzw"].push_back(pose.orientation.z);
  node["quaternion_xyzw"].push_back(pose.orientation.w);
  return node;
}

void writeYaml(const std::filesystem::path & path, const YAML::Node & node)
{
  std::ofstream file(path);
  file << node;
  if (!file.good()) {throw std::runtime_error("Cannot write " + path.string());}
}

void saveGrid(
  const std::filesystem::path & path, const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & grid,
  double now)
{
  YAML::Node node;
  node["available"] = static_cast<bool>(grid);
  if (grid) {
    node["frame"] = grid->header.frame_id;
    node["stamp"] = rclcpp::Time(grid->header.stamp).seconds();
    node["age_seconds"] = now - rclcpp::Time(grid->header.stamp).seconds();
    node["resolution"] = grid->info.resolution;
    node["width"] = grid->info.width;
    node["height"] = grid->info.height;
    node["origin"] = poseNode(grid->info.origin);
    node["data"].SetStyle(YAML::EmitterStyle::Flow);
    for (auto cell : grid->data) {node["data"].push_back(static_cast<int>(cell));}
  }
  writeYaml(path, node);
}
}  // namespace

NavigationEvidence::NavigationEvidence(rclcpp::Node & node) : node_(node)
{
  global_sub_ = node.create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/costmap", 1,
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {global_ = msg;});
  local_sub_ = node.create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", 1,
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {local_ = msg;});
  odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odom_ = msg;});
}

bool NavigationEvidence::stationary() const
{
  if (!odom_) {return false;}
  const double age = (node_.now() - rclcpp::Time(odom_->header.stamp)).seconds();
  const auto & v = odom_->twist.twist;
  return age >= -0.1 && age < 0.5 &&
    std::hypot(v.linear.x, v.linear.y) < 0.015 && std::abs(v.angular.z) < 0.03;
}

void NavigationEvidence::save(
  const std::filesystem::path & directory, const geometry_msgs::msg::PoseStamped & goal,
  const geometry_msgs::msg::PoseStamped & robot, const std::string & reason,
  const std::string & guard) const
{
  std::filesystem::create_directories(directory);
  YAML::Node node;
  node["stamp"] = node_.now().seconds();
  node["reason"] = reason;
  node["motion_guard"] = guard;
  node["goal_frame"] = goal.header.frame_id;
  node["goal"] = poseNode(goal.pose);
  node["robot_pose_available"] = !robot.header.frame_id.empty();
  node["robot_frame"] = robot.header.frame_id;
  node["robot_pose_stamp"] = rclcpp::Time(robot.header.stamp).seconds();
  node["robot"] = poseNode(robot.pose);
  node["stationary"] = stationary();
  if (odom_) {
    node["odom_pose"] = poseNode(odom_->pose.pose);
    node["odom_stamp"] = rclcpp::Time(odom_->header.stamp).seconds();
    node["linear_velocity"] = odom_->twist.twist.linear.x;
    node["angular_velocity"] = odom_->twist.twist.angular.z;
  }
  writeYaml(directory / "context.yaml", node);
  saveGrid(directory / "global_costmap.yaml", global_, node_.now().seconds());
  saveGrid(directory / "local_costmap.yaml", local_, node_.now().seconds());
}
}  // namespace cleanup_task_manager
