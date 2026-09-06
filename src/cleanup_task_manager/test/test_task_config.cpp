#include "cleanup_task_manager/task_config.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace
{

class TemporaryYaml
{
public:
  explicit TemporaryYaml(const std::string & contents)
  {
    path_ = (std::filesystem::temp_directory_path() /
      ("cleanup_task_config_" + std::to_string(counter_++) + ".yaml")).string();
    std::ofstream output(path_);
    output << contents;
  }

  ~TemporaryYaml()
  {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  const std::string & path() const
  {
    return path_;
  }

private:
  inline static int counter_{0};
  std::string path_;
};

TEST(TaskConfig, LoadsSafeDefaultsAndDropPose)
{
  TemporaryYaml yaml(R"(
cleanup:
  drop_pose: {configured: true, name: drop, x: 1.0, y: 2.0, yaw: 3.14,
    exclusion_radius: 0.45,
    placement: {x: 0.76, y: 2.0, floor_height: 0.0, release_height: 0.035}}
  scan: {turns: 4, turn_angle: -1.57, settle_seconds: 0.7,
    burst_frames: 8, min_confirmations: 3, capture_retries: 1}
  stations:
    - {name: scan_0, x: 1.0, y: 2.0, yaw: 0.0}
  association: {xy_gate: 0.25, min_confidence: 0.45}
  task_policy: {planner: gemini, allowed_classes: [banana]}
)");

  const auto config = cleanup_task_manager::TaskConfig::loadFromFile(yaml.path());
  EXPECT_TRUE(config.drop_pose_configured);
  EXPECT_DOUBLE_EQ(config.drop_pose.x, 1.0);
  EXPECT_DOUBLE_EQ(config.placement_point.x, 0.76);
  EXPECT_DOUBLE_EQ(config.placement_release_height, 0.035);
  ASSERT_EQ(config.stations.size(), 1U);
  EXPECT_EQ(config.stations.front().name, "scan_0");
  EXPECT_EQ(config.planner, "gemini");
}

TEST(TaskConfig, ExclusionUsesObjectLocationRatherThanBaseStop)
{
  cleanup_task_manager::TaskConfig config;
  config.drop_pose = {-0.16, 1.5, 3.141592653589793};
  config.placement_point = {-0.4, 1.5, 0.0};
  config.drop_exclusion_radius = 0.2;
  EXPECT_TRUE(config.isInDropZone(-0.4, 1.5));
  EXPECT_TRUE(config.isInDropZone(-0.39, 1.51));
  EXPECT_FALSE(config.isInDropZone(-0.16, 1.5));
  EXPECT_FALSE(config.isInDropZone(0.0, 1.5));
}

TEST(TaskConfig, RejectsInvalidDistanceRange)
{
  TemporaryYaml yaml(
    R"(
cleanup:
  target_min_distance: 2.0
  target_max_distance: 1.0
  drop_pose: {configured: false, x: 0.0, y: 0.0, yaw: 0.0,
    placement: {x: 0.24, y: 0.0}}
  scan: {turns: 4, turn_angle: -1.57, burst_frames: 8,
    min_confirmations: 3}
  stations:
    - {name: scan_0, x: 0.0, y: 0.0, yaw: 0.0}
  association: {xy_gate: 0.25, min_confidence: 0.45}
  task_policy: {planner: deterministic, allowed_classes: [banana]}
)");

  EXPECT_THROW(
    cleanup_task_manager::TaskConfig::loadFromFile(yaml.path()),
    std::runtime_error);
}

TEST(TaskConfig, RejectsDuplicateStationNames)
{
  TemporaryYaml yaml(R"(
cleanup:
  drop_pose: {configured: true, name: drop, x: 0.0, y: 0.0, yaw: 0.0,
    placement: {x: 0.24, y: 0.0}}
  scan: {turns: 4, turn_angle: -1.57, burst_frames: 8,
    min_confirmations: 3}
  stations:
    - {name: scan_0, x: 0.0, y: 0.0, yaw: 0.0}
    - {name: scan_0, x: 1.0, y: 0.0, yaw: 0.0}
  association: {xy_gate: 0.25, min_confidence: 0.45}
  task_policy: {planner: deterministic, allowed_classes: [banana]}
)");

  EXPECT_THROW(
    cleanup_task_manager::TaskConfig::loadFromFile(yaml.path()),
    std::runtime_error);
}

TEST(TaskConfig, RejectsImpossibleBurstConfirmation)
{
  TemporaryYaml yaml(R"(
cleanup:
  drop_pose: {configured: true, name: drop, x: 0.0, y: 0.0, yaw: 0.0,
    placement: {x: 0.24, y: 0.0}}
  scan: {turns: 4, turn_angle: -1.57, burst_frames: 2,
    min_confirmations: 3}
  stations:
    - {name: scan_0, x: 0.0, y: 0.0, yaw: 0.0}
  association: {xy_gate: 0.25, min_confidence: 0.45}
  task_policy: {planner: deterministic, allowed_classes: [banana]}
)");

  EXPECT_THROW(
    cleanup_task_manager::TaskConfig::loadFromFile(yaml.path()),
    std::runtime_error);
}

}  // namespace
