cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cleanup_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pick_and_place src/pick_and_place_node.cpp)
add_executable(candidate_grasp_plan src/candidate_grasp_plan.cpp)
target_include_directories(candidate_grasp_plan PRIVATE include)
target_include_directories(pick_and_place PRIVATE include)
ament_target_dependencies(pick_and_place
  rclcpp
  cleanup_interfaces
  rclcpp_action
  geometry_msgs
  std_srvs
  std_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  pick_and_place
  candidate_grasp_plan
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_pick_execution test/test_pick_execution.py TIMEOUT 90)
  ament_add_gtest(test_grasp_kinematics test/test_grasp_kinematics.cpp)
  target_include_directories(test_grasp_kinematics PRIVATE include)
endif()

ament_package()
