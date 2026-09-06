cmake_minimum_required(VERSION 3.8)
project(aruco_localizer)

# Keep real-time projection/IK optimized in a fresh workspace; Debug stays opt-in.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 검색
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(motion_guard_node src/motion_guard_node.cpp)
ament_target_dependencies(motion_guard_node rclcpp geometry_msgs std_msgs sensor_msgs
  tf2 tf2_ros tf2_geometry_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MarkerObservation.msg"
  DEPENDENCIES std_msgs
)

# Route configuration is kept as a ROS-independent library so parsing and
# validation can be tested without starting a node.
add_library(route_config src/route_config.cpp)
target_include_directories(route_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(route_config yaml-cpp)

# Marker correction acceptance is isolated from camera/TF plumbing so the
# near-field and motion gates can be regression tested independently.
add_library(localization_policy src/localization_policy.cpp)
target_include_directories(localization_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(motion_guard_node localization_policy)

add_library(marker_guidance src/marker_guidance.cpp)
target_include_directories(marker_guidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# -------------------------------------------------------------
# 1. 아루코 마커 기반 위치 추정 노드 (aruco_localizer_node)
# -------------------------------------------------------------
add_executable(aruco_localizer_node src/aruco_localizer_node.cpp)
target_include_directories(aruco_localizer_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(aruco_localizer_node
  localization_policy
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_eigen::tf2_eigen
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${OpenCV_LIBS}
  yaml-cpp
)
rosidl_get_typesupport_target(aruco_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(aruco_localizer_node "${aruco_typesupport_target}")

# -------------------------------------------------------------
# 2. 맵 PGM/YAML 및 마커 시각화 발행 노드 (map_marker_publisher_node)
# -------------------------------------------------------------
add_executable(map_marker_publisher_node src/map_marker_publisher_node.cpp)
target_include_directories(map_marker_publisher_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(map_marker_publisher_node
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${OpenCV_LIBS}
  yaml-cpp
)

# -------------------------------------------------------------
# 3. 아루코 마커 직선 경로 네비게이터 노드 (aruco_waypoint_navigator_node)
# -------------------------------------------------------------
add_executable(aruco_waypoint_navigator_node src/aruco_waypoint_navigator_node.cpp)
target_include_directories(aruco_waypoint_navigator_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(aruco_waypoint_navigator_node
  route_config
  marker_guidance
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${std_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)
target_link_libraries(
  aruco_waypoint_navigator_node "${aruco_typesupport_target}")

# -------------------------------------------------------------
# 4. Navigation event/image blackbox recorder
# -------------------------------------------------------------
add_executable(navigation_debug_recorder_node src/navigation_debug_recorder_node.cpp)
target_include_directories(navigation_debug_recorder_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(navigation_debug_recorder_node
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
)
target_link_libraries(
  navigation_debug_recorder_node "${aruco_typesupport_target}")

# -------------------------------------------------------------
# 설치 규칙
# -------------------------------------------------------------
# 1) 빌드된 C++ 실행 파일 설치
add_executable(obstacle_depth_node src/obstacle_depth_node.cpp)
target_include_directories(obstacle_depth_node PRIVATE include)
ament_target_dependencies(obstacle_depth_node rclcpp sensor_msgs std_msgs tf2 tf2_ros
  tf2_geometry_msgs)

install(TARGETS
  obstacle_depth_node
  motion_guard_node
  aruco_localizer_node
  map_marker_publisher_node
  aruco_waypoint_navigator_node
  navigation_debug_recorder_node
  route_config
  localization_policy
  marker_guidance
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
# launch, map, config 폴더를 모두 share 디렉터리로 복사
install(DIRECTORY launch map config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
# Python 스크립트 설치
install(PROGRAMS
  scripts/calibrate_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_carry_localization test/test_carry_localization.py TIMEOUT 30)
  ament_add_pytest_test(test_obstacle_depth_node test/test_obstacle_depth_node.py TIMEOUT 30)
  ament_add_pytest_test(test_navigation_trees test/test_navigation_trees.py)
  ament_add_pytest_test(test_motion_safety test/test_motion_safety.py TIMEOUT 60)

  ament_add_gtest(test_depth_projection test/test_depth_projection.cpp)
  target_include_directories(test_depth_projection PRIVATE include)

  ament_add_gtest(test_route_config test/test_route_config.cpp)
  target_compile_definitions(test_route_config PRIVATE
    ROUTES_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/routes.yaml"
  )
  target_link_libraries(test_route_config route_config)

  ament_add_gtest(test_localization_policy test/test_localization_policy.cpp)
  target_link_libraries(test_localization_policy localization_policy)

  ament_add_gtest(test_marker_guidance test/test_marker_guidance.cpp)
  target_link_libraries(test_marker_guidance marker_guidance)

  ament_add_gtest(test_nav2_config test/test_nav2_config.cpp)
  target_compile_definitions(test_nav2_config PRIVATE
    NAV2_PARAMS_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_params.yaml"
  )
  target_link_libraries(test_nav2_config yaml-cpp)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
