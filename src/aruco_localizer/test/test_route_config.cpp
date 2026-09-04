#include "aruco_localizer/route_config.hpp"

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
      ("aruco_route_config_" + std::to_string(counter_++) + ".yaml")).string();
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

TEST(RouteConfig, LoadsIndependentVirtualWaypoints)
{
  TemporaryYaml yaml(
    R"(
waypoints:
  corner:
    x: 1.25
    y: -0.75
    yaw: 1.5708
    role: turn
  scan_pose:
    x: 2.0
    y: 1.0
    yaw: 0.0
routes:
  cleanup_route: [corner, scan_pose]
)");

  const auto config = aruco_localizer::RouteConfig::loadFromFile(yaml.path());
  ASSERT_TRUE(config.hasRoute("cleanup_route"));
  ASSERT_EQ(config.route("cleanup_route").size(), 2U);
  EXPECT_EQ(config.route("cleanup_route")[0], "corner");
  EXPECT_DOUBLE_EQ(config.waypoint("corner").x, 1.25);
  EXPECT_EQ(config.waypoint("corner").role, "turn");
  EXPECT_EQ(config.waypoint("scan_pose").role, "transit");
}

TEST(RouteConfig, RejectsUnknownWaypointReference)
{
  TemporaryYaml yaml(
    R"(
waypoints:
  known: {x: 0.0, y: 0.0, yaw: 0.0}
routes:
  invalid: [missing]
)");

  EXPECT_THROW(
    aruco_localizer::RouteConfig::loadFromFile(yaml.path()),
    std::runtime_error);
}

TEST(RouteConfig, RejectsMissingPoseField)
{
  TemporaryYaml yaml(R"(
waypoints:
  invalid: {x: 0.0, y: 0.0}
routes:
  route: [invalid]
)");

  EXPECT_THROW(
    aruco_localizer::RouteConfig::loadFromFile(yaml.path()),
    std::runtime_error);
}

TEST(RouteConfig, RejectsNonFinitePose)
{
  TemporaryYaml yaml(
    R"(
waypoints:
  invalid: {x: .nan, y: 0.0, yaw: 0.0}
routes:
  route: [invalid]
)");

  EXPECT_THROW(
    aruco_localizer::RouteConfig::loadFromFile(yaml.path()),
    std::runtime_error);
}

}  // namespace
