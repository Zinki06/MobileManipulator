cmake_minimum_required(VERSION 3.8)
project(aruco_localizer)

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
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
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
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(aruco_waypoint_navigator_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${std_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
  yaml-cpp
)

# -------------------------------------------------------------
# 설치 규칙
# -------------------------------------------------------------
# 1) 빌드된 C++ 실행 파일 설치
install(TARGETS
  aruco_localizer_node
  map_marker_publisher_node
  aruco_waypoint_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)
# launch, map, config 폴더를 모두 share 디렉터리로 복사
install(DIRECTORY launch map config
  DESTINATION share/${PROJECT_NAME}
)
# Python 스크립트 설치
install(PROGRAMS
  scripts/calibrate_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
