#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

#ifndef NAV2_PARAMS_PATH
#error "NAV2_PARAMS_PATH must point to the package Nav2 configuration"
#endif

namespace
{

TEST(Nav2Config, StationControllerUsesHardwareExecutableSpeeds)
{
  const YAML::Node config = YAML::LoadFile(NAV2_PARAMS_PATH);
  const YAML::Node controller =
    config["controller_server"]["ros__parameters"];

  EXPECT_EQ(controller["controller_plugins"].as<std::vector<std::string>>(),
    (std::vector<std::string>{"StationPath", "ApproachPath"}));
  for (const std::string name : {"StationPath", "ApproachPath"}) {
    EXPECT_GE(controller[name]["min_approach_linear_velocity"].as<double>(), 0.02);
    EXPECT_LE(controller[name]["desired_linear_vel"].as<double>(), 0.10);
    EXPECT_EQ(controller[name]["plugin"].as<std::string>(),
      "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");
    EXPECT_TRUE(controller[name]["use_collision_detection"].as<bool>());
  }
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

TEST(Nav2Config, ManipulationHasIndependentPreciseController)
{
  const auto config = YAML::LoadFile(NAV2_PARAMS_PATH)["controller_server"]["ros__parameters"];
  EXPECT_DOUBLE_EQ(config["general_goal_checker"]["xy_goal_tolerance"].as<double>(), 0.12);
  EXPECT_LE(config["manipulation_goal_checker"]["xy_goal_tolerance"].as<double>(), 0.02);
  EXPECT_LE(config["ApproachPath"]["desired_linear_vel"].as<double>(), 0.06);
  EXPECT_TRUE(config["ApproachPath"]["use_collision_detection"].as<bool>());
}

TEST(Nav2Config, ClearedAreaUsesStaticMapWithoutLiveObstacleDetours)
{
  const auto config = YAML::LoadFile(NAV2_PARAMS_PATH);
  for (const std::string name : {"global_costmap", "local_costmap"}) {
    const auto costmap = config[name][name]["ros__parameters"];
    EXPECT_EQ(costmap["plugins"].as<std::vector<std::string>>(),
      (std::vector<std::string>{"static_layer", "inflation_layer"}));
    EXPECT_FALSE(costmap["obstacle_layer"]);
    EXPECT_EQ(costmap["static_layer"]["map_topic"].as<std::string>(), "/map");
    EXPECT_TRUE(costmap["static_layer"]["map_subscribe_transient_local"].as<bool>());
    EXPECT_TRUE(costmap["track_unknown_space"].as<bool>());
    EXPECT_GT(costmap["robot_radius"].as<double>(), 0.0);
    EXPECT_GT(costmap["inflation_layer"]["inflation_radius"].as<double>(), 0.0);
  }
  EXPECT_FALSE(config["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"].as<bool>());
}

}  // namespace
