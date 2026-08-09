# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target turtlebot3_pick_place::turtlebot3_pick_place
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${turtlebot3_pick_place_TARGETS}.
if(turtlebot3_pick_place_TARGETS AND NOT TARGET turtlebot3_pick_place::turtlebot3_pick_place)
  add_library(turtlebot3_pick_place::turtlebot3_pick_place INTERFACE IMPORTED)
  set_target_properties(turtlebot3_pick_place::turtlebot3_pick_place PROPERTIES
    INTERFACE_LINK_LIBRARIES "${turtlebot3_pick_place_TARGETS}")
endif()
