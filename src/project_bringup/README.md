# project_bringup

This package starts the TurtleBot3 hardware, RealSense perception pipeline,
ArUco/Nav2 navigation, segmentation, and pick-and-place node from one terminal.
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
`start_navigation`, and `start_pick_and_place`. Set `use_fake_hardware:=true`
to use fake ros2_control hardware; the existing hardware launch still starts
the configured lidar.

## Execute a grasp

The pick-and-place node starts safely without moving the arm. After a valid
`/object_centroid` is available, trigger one sequence from another ROS 2
terminal or a higher-level controller:

```bash
ros2 service call /execute_pick_and_place std_srvs/srv/Trigger "{}"
```

Concurrent requests are rejected. A failed request returns the reason in the
service response.

## Remote RViz

RViz is intentionally disabled on the robot computer. On a remote computer
using the same `ROS_DOMAIN_ID` and workspace, run:

```bash
rviz2 -d "$(ros2 pkg prefix --share \
  turtlebot3_manipulation_navigation2)/rviz/navigation2.rviz"
```

Set `LDS_MODEL` before launching if the robot uses a lidar other than the
default selected by `turtlebot3_manipulation_bringup`.
