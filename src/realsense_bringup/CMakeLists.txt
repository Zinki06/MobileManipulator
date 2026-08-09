cmake_minimum_required(VERSION 3.8)
project(realsense_bringup)

# ----------------------------------------------------
# 컴파일 옵션
# ----------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------------------------------
# ROS 2 필수 패키지
# ----------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)

# ----------------------------------------------------
# OpenCV / cv_bridge
# ----------------------------------------------------
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ----------------------------------------------------
# TF2 관련
# ----------------------------------------------------
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

# ----------------------------------------------------
# PCL, Eigen 관련 (중복 선언 통합)
# ----------------------------------------------------
# common, io, filters, segmentation, kdtree, search를 한 번에 선언합니다.
find_package(PCL REQUIRED COMPONENTS common io filters segmentation kdtree search)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

# PCL 매크로 정의 추가 (컴파일 에러 방지)
add_definitions(${PCL_DEFINITIONS})

# ----------------------------------------------------
# include 디렉토리 지정
# ----------------------------------------------------
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ----------------------------------------------------
# 실행 파일
# ----------------------------------------------------
add_executable(realsense_tf_node src/realsense_tf.cpp)

# ----------------------------------------------------
# 의존성 및 라이브러리 링크
# ----------------------------------------------------
ament_target_dependencies(realsense_tf_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  cv_bridge
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
)

# OpenCV, PCL, Eigen 직접 링크
target_link_libraries(realsense_tf_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# ----------------------------------------------------
# 설치 규칙
# ----------------------------------------------------
install(TARGETS
  realsense_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------------------------------
# 테스트 관련
# ----------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ----------------------------------------------------
# 의존성 export
# ----------------------------------------------------
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  cv_bridge
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_conversions
)

# ----------------------------------------------------
# 패키지 선언
# ----------------------------------------------------
ament_package()