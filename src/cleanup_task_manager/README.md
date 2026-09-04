# cleanup_task_manager

This package owns mission-level sequencing only. Localization remains in
`aruco_localizer`, base motion remains in Nav2, and arm motion remains in
`pick_and_place`.

## State flow

```text
PATROLLING -> STOPPING_PATROL -> NAVIGATING_TO_OBJECT -> PICKING
           -> NAVIGATING_TO_DROP -> RELEASING -> PARKING -> PATROLLING
```

The manager accepts fresh `/object_centroid` messages only while patrolling.
It transforms the selected point into `map`, computes a configurable standoff
pose, waits until route navigation is idle, and then owns the Nav2 goal. Before
calling the manipulation service it publishes the selected map target to
`/cleanup/pick_target`, so later perception updates cannot silently replace the
mission target.

## Safety configuration

`config/task_zones.yaml` ships with `drop_pose.configured: false`. Set the pose
and enable it only after manually verifying that the full robot and arm have
clearance. `/start_cleanup` rejects requests until this is done and localization
is neither `UNINITIALIZED` nor `DEGRADED`.

Park the robot at the intended collection pose and inspect it with
`ros2 run tf2_ros tf2_echo map base_link`. Copy its `x`, `y`, and yaw into the
configuration, rebuild, and verify the pose once with Nav2 before enabling a
full cleanup cycle.

Stopping cleanup cancels base navigation and patrol. The current manipulation
service is not preemptible, so an arm sequence already in progress completes on
the `pick_and_place` node.
