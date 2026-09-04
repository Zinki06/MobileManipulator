#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>

#ifndef NAV2_PARAMS_PATH
#error "NAV2_PARAMS_PATH must point to the package Nav2 configuration"
#endif

namespace
{

TEST(Nav2Config, RotateOnlyWindowCannotDeadlockGoalChecker)
{
  const YAML::Node config = YAML::LoadFile(NAV2_PARAMS_PATH);
  const YAML::Node controller =
    config["controller_server"]["ros__parameters"];

  const double goal_tolerance =
    controller["general_goal_checker"]["xy_goal_tolerance"].as<double>();
  const double rotate_only_tolerance =
    controller["FollowPath"]["xy_goal_tolerance"].as<double>();

  EXPECT_GT(goal_tolerance, 0.0);
  EXPECT_GT(rotate_only_tolerance, 0.0);
  EXPECT_LE(rotate_only_tolerance, goal_tolerance);
}

TEST(Nav2Config, ProgressCheckerCountsAngularMotion)
{
  const YAML::Node config = YAML::LoadFile(NAV2_PARAMS_PATH);
  const YAML::Node progress =
    config["controller_server"]["ros__parameters"]["progress_checker"];

  EXPECT_EQ(
    progress["plugin"].as<std::string>(),
    "nav2_controller::PoseProgressChecker");
  EXPECT_GT(progress["required_movement_angle"].as<double>(), 0.0);
}

}  // namespace
