# Landmark localization, virtual routes, and cleanup tasks

Detailed navigation implementation, failure analysis, and hardware evidence:
[`virtual_waypoint_navigation_implementation.md`](virtual_waypoint_navigation_implementation.md)

## Responsibility boundaries

- `aruco_localizer_node` estimates and continuously publishes `map -> odom`.
- `aruco_waypoint_navigator_node` executes named virtual base poses through
  Nav2. It neither subscribes to marker status nor publishes `/cmd_vel`.
- `navigation_debug_recorder_node` owns camera snapshots and filesystem logs.
- `cleanup_task_manager_node` coordinates patrol, object approach, pickup, drop,
  and patrol resume.
- `pick_and_place` owns manipulator and gripper motion.

## Data ownership

- `map/new_map_markers.yaml`: surveyed physical ArUco landmark transforms.
- `config/routes.yaml`: robot base poses, including safe corner pivots and yaw.
- `cleanup_task_manager/config/task_zones.yaml`: task policy and collection pose.

A physical marker pose must never be reused implicitly as a base waypoint or an
object interaction pose.

## Localization lifecycle

- `UNINITIALIZED`: no valid global correction has ever been accepted.
- `MARKER_CORRECTED`: a valid correction was accepted recently.
- `DEAD_RECKONING`: the last correction is frozen and live odometry advances
  the robot pose.
- `DEGRADED`: configured elapsed-time or traveled-distance limits were exceeded.

Marker corrections are accepted only while the base angular speed is below the
configured threshold and the marker is outside the near-field floor-plane
radius. The near-field check is evaluated in `base_link`; camera height is not
allowed to make an unsafe close observation look far away. Rejected
observations leave the last `map -> odom` transform frozen, so odometry remains
the sole short-term motion source during an in-place turn.

Route execution only requires an initialized `map -> base_link` transform. It
does not pause when the mode changes from `MARKER_CORRECTED` to
`DEAD_RECKONING`. Higher-level cleanup startup rejects `DEGRADED` localization;
an already-running route remains under Nav2 control instead of stopping merely
because a marker left the camera view.

## Hardware validation order

1. Verify `to_5` and `to_0` without objects.
2. Cover all markers after the initial correction and verify continued motion.
3. Measure corner position and yaw error; teach `routes.yaml` where needed.
4. Reveal a marker mid-route and confirm that the correction is smooth.
5. Configure and validate the collection pose with teleoperation.
6. Test one stationary object before enabling repeated cleanup patrol.

The checked-in route coordinates preserve the former map geometry as initial
values only. Before task testing, place the base on each intended safe pivot
(the green points in the floor plan) and record those poses with
`calibrate_waypoints.py`; do not copy the neighboring marker coordinates.
