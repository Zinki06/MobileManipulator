#include "cleanup_task_manager/task_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace cleanup_task_manager
{

namespace
{

double finiteValue(
  const YAML::Node & node, const std::string & field, double default_value)
{
  const double value = node[field] ? node[field].as<double>() : default_value;
  if (!std::isfinite(value)) {
    throw std::runtime_error("Task config field '" + field + "' must be finite");
  }
  return value;
}

int integerValue(
  const YAML::Node & node, const std::string & field, int default_value)
{
  return node[field] ? node[field].as<int>() : default_value;
}

TaskPose poseValue(const YAML::Node & node, const std::string & context)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error(context + " must be a map");
  }
  TaskPose pose;
  pose.x = finiteValue(node, "x", 0.0);
  pose.y = finiteValue(node, "y", 0.0);
  pose.yaw = finiteValue(node, "yaw", 0.0);
  return pose;
}

}  // namespace

TaskConfig TaskConfig::loadFromFile(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node cleanup = root["cleanup"];
  if (!cleanup || !cleanup.IsMap()) {
    throw std::runtime_error("Task config must contain a 'cleanup' map");
  }

  TaskConfig config;
  config.target_min_distance = finiteValue(
    cleanup, "target_min_distance", config.target_min_distance);
  config.target_max_distance = finiteValue(
    cleanup, "target_max_distance", config.target_max_distance);

  const YAML::Node drop_pose = cleanup["drop_pose"];
  if (!drop_pose || !drop_pose.IsMap()) {
    throw std::runtime_error("Task config must contain cleanup.drop_pose");
  }
  config.drop_pose_configured =
    drop_pose["configured"] && drop_pose["configured"].as<bool>();
  config.drop_pose = poseValue(drop_pose, "cleanup.drop_pose");
  config.drop_zone_name = drop_pose["name"] ?
    drop_pose["name"].as<std::string>() : config.drop_zone_name;
  config.drop_exclusion_radius = finiteValue(
    drop_pose, "exclusion_radius", config.drop_exclusion_radius);
  const auto placement = drop_pose["placement"];
  if (!placement || !placement.IsMap() || !placement["x"] || !placement["y"]) {
    throw std::runtime_error("cleanup.drop_pose.placement requires an explicit object x/y");
  }
  config.placement_point = poseValue(placement, "cleanup.drop_pose.placement");
  config.placement_floor_height = finiteValue(placement, "floor_height", 0.0);
  config.placement_release_height = finiteValue(placement, "release_height", 0.035);
  const double dx = config.placement_point.x - config.drop_pose.x;
  const double dy = config.placement_point.y - config.drop_pose.y;
  const double forward = dx * std::cos(config.drop_pose.yaw) + dy * std::sin(config.drop_pose.yaw);
  const double sideways = -dx * std::sin(config.drop_pose.yaw) + dy * std::cos(config.drop_pose.yaw);
  if (forward < 0.15 || forward > 0.30 || std::abs(sideways) > 0.03 ||
    config.placement_release_height < 0.025 || config.placement_release_height > 0.06)
  {
    throw std::runtime_error("Placement must be 15-30cm ahead of the base, with a 25-60mm release height");
  }

  const YAML::Node scan = cleanup["scan"];
  if (!scan || !scan.IsMap()) {
    throw std::runtime_error("Task config must contain cleanup.scan");
  }
  config.scan.turns = integerValue(scan, "turns", config.scan.turns);
  config.scan.turn_angle = finiteValue(
    scan, "turn_angle", config.scan.turn_angle);
  config.scan.settle_seconds = finiteValue(
    scan, "settle_seconds", config.scan.settle_seconds);
  config.scan.burst_frames = integerValue(
    scan, "burst_frames", config.scan.burst_frames);
  config.scan.min_confirmations = integerValue(
    scan, "min_confirmations", config.scan.min_confirmations);
  config.scan.capture_retries = integerValue(
    scan, "capture_retries", config.scan.capture_retries);

  const YAML::Node stations = cleanup["stations"];
  if (!stations || !stations.IsSequence() || stations.size() == 0U) {
    throw std::runtime_error("cleanup.stations must be a non-empty sequence");
  }
  std::unordered_set<std::string> station_names;
  for (const YAML::Node & station_node : stations) {
    if (!station_node.IsMap() || !station_node["name"]) {
      throw std::runtime_error("Every cleanup station requires a name");
    }
    TaskStation station;
    station.name = station_node["name"].as<std::string>();
    if (station.name.empty() || !station_names.insert(station.name).second) {
      throw std::runtime_error("Cleanup station names must be non-empty and unique");
    }
    station.pose = poseValue(station_node, "cleanup station '" + station.name + "'");
    config.stations.push_back(station);
  }

  const YAML::Node association = cleanup["association"];
  if (!association || !association.IsMap()) {
    throw std::runtime_error("Task config must contain cleanup.association");
  }
  config.association_xy_gate = finiteValue(
    association, "xy_gate", config.association_xy_gate);
  config.min_confidence = finiteValue(
    association, "min_confidence", config.min_confidence);

  const YAML::Node policy = cleanup["task_policy"];
  if (!policy || !policy.IsMap()) {
    throw std::runtime_error("Task config must contain cleanup.task_policy");
  }
  config.planner = policy["planner"] ?
    policy["planner"].as<std::string>() : config.planner;
  if (policy["allowed_classes"]) {
    if (!policy["allowed_classes"].IsSequence()) {
      throw std::runtime_error("task_policy.allowed_classes must be a sequence");
    }
    config.allowed_classes.clear();
    for (const YAML::Node & class_node : policy["allowed_classes"]) {
      config.allowed_classes.push_back(class_node.as<std::string>());
    }
  }

  if (config.target_min_distance < 0.0 ||
    config.target_max_distance <= config.target_min_distance ||
    config.drop_exclusion_radius <= 0.0 ||
    config.association_xy_gate <= 0.0 || config.min_confidence < 0.0 ||
    config.min_confidence > 1.0)
  {
    throw std::runtime_error("Task config distance and time limits are invalid");
  }
  if (config.scan.turns < 1 || config.scan.turns > 16 ||
    std::abs(config.scan.turn_angle) < 0.01 ||
    std::abs(config.scan.turn_angle) > 3.141592653589793 ||
    config.scan.settle_seconds < 0.0 || config.scan.burst_frames < 1 ||
    config.scan.burst_frames > 100 || config.scan.min_confirmations < 1 ||
    config.scan.min_confirmations > config.scan.burst_frames ||
    config.scan.capture_retries < 0 || config.scan.capture_retries > 5)
  {
    throw std::runtime_error("Task config scan settings are invalid");
  }
  if (config.drop_zone_name.empty() || config.allowed_classes.empty() ||
    (config.planner != "gemini" && config.planner != "deterministic"))
  {
    throw std::runtime_error("Task policy or drop zone name is invalid");
  }
  return config;
}

}  // namespace cleanup_task_manager
