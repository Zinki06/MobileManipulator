cmake_minimum_required(VERSION 3.8)
project(project_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/record_debug.py DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_feedback_config test/test_feedback_config.py)
  ament_add_pytest_test(test_debug_recording test/test_debug_recording.py TIMEOUT 60)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
