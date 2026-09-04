# project_bringup

This package starts the TurtleBot3 hardware, RealSense perception pipeline,
ArUco/Nav2 navigation, segmentation, pick-and-place, and the cleanup task
manager from one terminal.
It reuses the existing package launch files without modifying them.

## Build and run

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to project_bringup
source install/setup.bash
ros2 launch project_bringup project.launch.py
```

Stop the complete stack with `Ctrl+C`. All components start by default. Disable
individual groups when debugging, for example:

```bash
ros2 launch project_bringup project.launch.py \
  start_segmentation:=false start_pick_and_place:=false
```

Available switches are `start_robot`, `start_realsense`, `start_segmentation`,
`start_navigation`, `start_pick_and_place`, and `start_cleanup_manager`. Set
`use_fake_hardware:=true` to use fake ros2_control hardware; the existing
hardware launch still starts the configured lidar.

## Virtual waypoint navigation

The complete implementation, the 25 cm rotation-deadlock analysis, and hardware
validation evidence are documented in
[`docs/virtual_waypoint_navigation_implementation.md`](../../docs/virtual_waypoint_navigation_implementation.md).

Physical marker landmarks live in `aruco_localizer/map/new_map_markers.yaml`.
Robot base waypoints and their final yaw live separately in
`aruco_localizer/config/routes.yaml`. Marker visibility is not a condition for
advancing a route after localization has been initialized.

The compatibility service names remain available:

```bash
ros2 service call /navigate_to_marker_5 std_srvs/srv/Trigger "{}"
ros2 service call /navigate_to_marker_0 std_srvs/srv/Trigger "{}"
```

Teach a virtual waypoint by moving the robot to a safe pose and running:

```bash
ros2 run aruco_localizer calibrate_waypoints.py
```

## Execute a grasp

In the integrated launch, the task manager selects `/object_centroid` and
publishes the chosen immutable target on `/cleanup/pick_target`. It then
triggers the pick sequence. Standalone use can retain the default
`/object_centroid` target topic.

```bash
ros2 service call /execute_pick_and_place std_srvs/srv/Trigger "{}"
```

Concurrent requests are rejected. A failed request returns the reason in the
service response.

## Cleanup mission

Before enabling cleanup, teach a collision-free base pose beside the collection
bin in `cleanup_task_manager/config/task_zones.yaml`, then set
`drop_pose.configured: true`. This explicit gate prevents motion toward the
placeholder coordinate.

Use the following command while the robot is parked at the intended bin pose,
then copy the translation and yaw into `task_zones.yaml`:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

```bash
ros2 service call /start_cleanup std_srvs/srv/Trigger "{}"
ros2 service call /stop_cleanup std_srvs/srv/Trigger "{}"
ros2 topic echo /cleanup/status
```

The mission state machine performs patrol, target selection, Nav2 approach,
pickup, travel to the configured drop pose, release, arm parking, and patrol
resume. It does not require an ArUco marker at the object or drop position.

## Remote RViz

RViz is intentionally disabled on the robot computer. On a remote computer
using the same `ROS_DOMAIN_ID` and workspace, run:

```bash
rviz2 -d "$(ros2 pkg prefix --share \
  turtlebot3_manipulation_navigation2)/rviz/navigation2.rviz"
```

Set `LDS_MODEL` before launching if the robot uses a lidar other than the
default selected by `turtlebot3_manipulation_bringup`.
