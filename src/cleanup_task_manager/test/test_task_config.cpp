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
  drop_pose: {configured: true, x: 1.0, y: 2.0, yaw: 3.14}
)");

  const auto config = cleanup_task_manager::TaskConfig::loadFromFile(yaml.path());
  EXPECT_TRUE(config.drop_pose_configured);
  EXPECT_DOUBLE_EQ(config.drop_pose.x, 1.0);
  EXPECT_DOUBLE_EQ(config.approach_standoff, 0.24);
}

TEST(TaskConfig, RejectsInvalidDistanceRange)
{
  TemporaryYaml yaml(
    R"(
cleanup:
  target_min_distance: 2.0
  target_max_distance: 1.0
  drop_pose: {configured: false, x: 0.0, y: 0.0, yaw: 0.0}
)");

  EXPECT_THROW(
    cleanup_task_manager::TaskConfig::loadFromFile(yaml.path()),
    std::runtime_error);
}

}  // namespace
