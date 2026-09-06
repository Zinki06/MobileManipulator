#ifndef CLEANUP_TASK_MANAGER__TASK_CONFIG_HPP_
#define CLEANUP_TASK_MANAGER__TASK_CONFIG_HPP_

#include <string>
#include <vector>
#include <cmath>

namespace cleanup_task_manager
{

struct TaskPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct TaskStation
{
  std::string name;
  TaskPose pose;
};

struct ScanConfig
{
  int turns{4};
  double turn_angle{-1.5707963267948966};
  double settle_seconds{0.7};
  int burst_frames{8};
  int min_confirmations{3};
  int capture_retries{1};
};

struct TaskConfig
{
  bool drop_pose_configured{false};
  TaskPose drop_pose;
  std::string drop_zone_name{"marker_3_outer_collection"};
  TaskPose placement_point;
  double placement_floor_height{0.0};
  double placement_release_height{0.035};
  double drop_exclusion_radius{0.45};
  std::vector<TaskStation> stations;
  ScanConfig scan;
  double association_xy_gate{0.25};
  double min_confidence{0.45};
  std::string planner{"gemini"};
  std::vector<std::string> allowed_classes{"banana"};
  double target_min_distance{0.15};
  double target_max_distance{2.0};

  static TaskConfig loadFromFile(const std::string & path);

  bool isInDropZone(double x, double y) const
  {
    return std::hypot(x - placement_point.x, y - placement_point.y) <= drop_exclusion_radius;
  }
};

}  // namespace cleanup_task_manager

#endif  // CLEANUP_TASK_MANAGER__TASK_CONFIG_HPP_
