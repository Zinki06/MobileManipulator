cmake_minimum_required(VERSION 3.8)
project(pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pick_and_place src/pick_and_place_node.cpp)
ament_target_dependencies(pick_and_place
  rclcpp
  rclcpp_action
  geometry_msgs
  std_srvs
  trajectory_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  pick_and_place
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
