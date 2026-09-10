cmake_minimum_required(VERSION 3.8)
project(cleanup_task_manager)

# Keep real-time projection/IK optimized in a fresh workspace; Debug stays opt-in.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cleanup_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(task_config src/task_config.cpp)
target_include_directories(task_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(task_config yaml-cpp)

add_executable(cleanup_task_manager_node
  src/cleanup_task_manager_node.cpp src/navigation_evidence.cpp)
target_include_directories(cleanup_task_manager_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(cleanup_task_manager_node
  rclcpp
  rclcpp_action
  cleanup_interfaces
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(cleanup_task_manager_node task_config)

install(TARGETS
  cleanup_task_manager_node
  task_config
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_task_config test/test_task_config.cpp)
  target_link_libraries(test_task_config task_config)
  ament_add_pytest_test(test_mission_flow test/test_mission_flow.py TIMEOUT 300)
endif()

ament_package()
