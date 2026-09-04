#include "cleanup_task_manager/task_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>

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

}  // namespace

TaskConfig TaskConfig::loadFromFile(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node cleanup = root["cleanup"];
  if (!cleanup || !cleanup.IsMap()) {
    throw std::runtime_error("Task config must contain a 'cleanup' map");
  }

  TaskConfig config;
  config.approach_standoff = finiteValue(
    cleanup, "approach_standoff", config.approach_standoff);
  config.target_max_age = finiteValue(
    cleanup, "target_max_age", config.target_max_age);
  config.target_min_distance = finiteValue(
    cleanup, "target_min_distance", config.target_min_distance);
  config.target_max_distance = finiteValue(
    cleanup, "target_max_distance", config.target_max_distance);
  config.target_cooldown = finiteValue(
    cleanup, "target_cooldown", config.target_cooldown);

  const YAML::Node drop_pose = cleanup["drop_pose"];
  if (!drop_pose || !drop_pose.IsMap()) {
    throw std::runtime_error("Task config must contain cleanup.drop_pose");
  }
  config.drop_pose_configured =
    drop_pose["configured"] && drop_pose["configured"].as<bool>();
  config.drop_pose.x = finiteValue(drop_pose, "x", 0.0);
  config.drop_pose.y = finiteValue(drop_pose, "y", 0.0);
  config.drop_pose.yaw = finiteValue(drop_pose, "yaw", 0.0);

  if (config.approach_standoff <= 0.0 || config.target_max_age <= 0.0 ||
    config.target_min_distance < 0.0 ||
    config.target_max_distance <= config.target_min_distance ||
    config.target_cooldown < 0.0)
  {
    throw std::runtime_error("Task config distance and time limits are invalid");
  }
  return config;
}

}  // namespace cleanup_task_manager
