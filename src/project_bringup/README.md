# project_bringup

This package starts the TurtleBot3 hardware, RealSense perception pipeline,
ArUco/Nav2 navigation, request-driven cleanup perception, the constrained
Gemini planner, pick-and-place, and the cleanup task manager from one terminal.
It reuses existing descriptions and hardware without modifying vendor packages.
The project-owned `feedback_robot.launch.py` applies encoder-feedback odometry
and arm feedback to a generated controller YAML. Turn scale defaults to 1.0.
All Nav2 velocity producers go through collision monitoring and a final motion
guard; large localization corrections latch a stop. See
[collision and grasp fixes](../../docs/COLLISION_AND_GRASP_FIX.md).

The default planner model is `gemini-3.5-flash-lite`; override it with
`gemini_model:=...`. The workspace `.env` supplies `GEMINI_API_KEY` only to the
planner. Object approaches use a separate precise Nav2 controller and the
existing arm's IK through `/cleanup/evaluate_grasp`; ordinary station navigation
retains its original controller and tolerance.

## Build and run

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to project_bringup
source install/setup.bash
ros2 launch project_bringup project.launch.py
```

Stop the complete stack with `Ctrl+C`. The legacy segmentation tracker is off
by default because cleanup perception loads the existing YOLO and SAM2 weights
on demand. YOLO confirms all banana candidates across a burst; SAM2 runs only
on confirmed boxes to refine the final grasp depth without loading a duplicate
continuous tracker.
Disable individual groups when debugging, for example:

```bash
ros2 launch project_bringup project.launch.py \
  start_segmentation:=false start_pick_and_place:=false
```

Available switches also include `start_cleanup_perception` and
`start_cleanup_planner` in addition to `start_robot`, `start_realsense`,
`start_segmentation`, `start_navigation`, `start_pick_and_place`, and
`start_cleanup_manager`. Set `use_fake_hardware:=true` to use fake ros2_control
hardware; the existing hardware launch still starts the configured lidar.

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

### Station scan navigation dry run

The dry run visits `scan_station_0` through `scan_station_5`. At every station
it uses the Nav2 Spin action to turn right 90 degrees four times, waits 700 ms
after every turn, and emits `SCAN_HEADING_READY` before continuing. Perception,
the arm, and cleanup sequencing are not required for this test.

The following describes the earlier open-loop experiment (superseded by the
encoder-feedback bringup above). The base controller formerly published open-loop odometry. Hardware evidence
showed that a commanded 90-degree turn produced about 75 degrees of physical
rotation, so the scan navigator applies a configurable `spin_command_scale` of
`1.20`. After a station scan it first uses route geometry and a short symmetric
camera search to find the next marker. If the marker is clipped or too close to
produce a correction, the Nav2 BackUp behavior moves the robot backward by
12 cm and retries, up to two times. A stable observation of the expected marker
corrects localization; other IDs are geometry hints only. Reacquisition is a
bounded best-effort operation: if no usable marker is found, the robot preserves
the last `map -> odom` transform and continues to the next station on wheel
odometry instead of aborting or spinning indefinitely.

Motion execution now goes through `robot_motion` for both patrol and cleanup.
A failed encoder turn stops the mission; it does not trigger a map-frame
recenter move or an extra turn retry. See
[MOTION_ARCHITECTURE.md](../../../docs/MOTION_ARCHITECTURE.md) for safety waits,
hardware-aware controllers, diagnostic records, and field-test limits.

Start only the robot, camera/localization, and navigation components:

```bash
ros2 launch project_bringup project.launch.py \
  start_segmentation:=false \
  start_pick_and_place:=false \
  start_cleanup_manager:=false
```

Then start the one-way station test:

```bash
ros2 service call /start_station_scan_test std_srvs/srv/Trigger "{}"
```

Stop it with `/stop_marker_patrol`. The scan poses in `routes.yaml` initially
match the marker-map geometry and must be taught as safe base poses before use
near furniture or objects.

Teach a virtual waypoint by moving the robot to a safe pose and running:

```bash
ros2 run aruco_localizer calibrate_waypoints.py
```

## Execute a grasp

In the integrated launch, `cleanup_perception` confirms a YOLO candidate and
refines its depth with SAM2. The task manager reserves that UUID, reacquires it
after approach, and publishes the fresh target on `/cleanup/pick_target` before
triggering the candidate body pick sequence. The integrated launch enables
`grasp_camera_20260906.yaml`, body-section/pitch selection, and a 30mm link1 +X
correction. Perception publishes the uncorrected observed body point; the pick
node applies the correction exactly once. The manager refuses a legacy or
navigation-only candidate at execution. Startup `GRASP_CONFIG` and each `PLAN`
event record the active strategy and correction. Restart the whole bringup after
rebuilding because the observation/evaluation ROS interfaces changed.
Standalone `pick_and_place` retains its legacy defaults for compatibility.

```bash
ros2 service call /execute_pick_and_place std_srvs/srv/Trigger "{}"
```

Concurrent requests are rejected. A failed request returns the reason in the
service response.

## Cleanup mission

The banana cleanup scans stations 0 through 5 at eight headings separated by
45 degrees, confirming detections over 3–5 frames. Empty headings save an RGB
image and metadata after all five frames, so missed objects can be investigated.
The collection point is outside ID 3: object map position (-0.40, 1.50), base
stop (-0.16, 1.50, yaw=pi). The 40cm distance is provisional; see
`cleanup_task_manager/config/task_zones.yaml`. Objects are lowered before release.
When measuring a replacement base pose, use:

```bash
ros2 run tf2_ros tf2_echo map base_link
```

```bash
# /home/user/turtlebot3_ws/.env 안에 입력
GEMINI_API_KEY=YOUR_API_KEY

ros2 launch project_bringup project.launch.py
```

In a second terminal:

```bash
source /home/user/turtlebot3_ws/install/setup.bash
ros2 service call /start_cleanup std_srvs/srv/Trigger "{}"
ros2 service call /stop_cleanup std_srvs/srv/Trigger "{}"
ros2 topic echo /cleanup/status
```

Gemini receives only validated object UUIDs and an action allowlist. Its output
must pass a strict schema and manager-side validation. A missing key, timeout,
invalid JSON, unknown UUID, or unknown action uses a deterministic banana
fallback, so cloud availability does not own the safety path. The mission does
not require an ArUco marker at the object or drop position. Evidence is written
under `cleanup_debug/run_YYYYMMDD_HHMMSS_PID/`.

## Remote RViz

RViz is intentionally disabled on the robot computer. On a remote computer
using the same `ROS_DOMAIN_ID` and workspace, run:

```bash
rviz2 -d "$(ros2 pkg prefix --share \
  turtlebot3_manipulation_navigation2)/rviz/navigation2.rviz"
```

Set `LDS_MODEL` before launching if the robot uses a lidar other than the
default selected by `turtlebot3_manipulation_bringup`.
