#ifndef CLEANUP_TASK_MANAGER__TASK_CONFIG_HPP_
#define CLEANUP_TASK_MANAGER__TASK_CONFIG_HPP_

#include <string>

namespace cleanup_task_manager
{

struct TaskPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TaskConfig
{
  bool drop_pose_configured{false};
  TaskPose drop_pose;
  double approach_standoff{0.24};
  double target_max_age{1.0};
  double target_min_distance{0.15};
  double target_max_distance{2.0};
  double target_cooldown{5.0};

  static TaskConfig loadFromFile(const std::string & path);
};

}  // namespace cleanup_task_manager

#endif  // CLEANUP_TASK_MANAGER__TASK_CONFIG_HPP_
