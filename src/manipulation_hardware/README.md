# Manipulation hardware

Independent ros2_control plugin `manipulation_hardware/OpenCRSystem`. It owns the
shared OpenCR USB port for the wheels, arm and gripper. Do not run another hardware
driver or raw serial probe on that port at the same time.

The transport was derived from the ROBOTIS reference driver (see NOTICE). The
reference `src/turtlebot3_manipulation` tree is not modified or linked as a driver.

Changes:

- Keep the heartbeat alive throughout IMU calibration so firmware accepts setup.
- Seed initial commands from measured positions before torque enable.
- Check both command staging and commit-trigger writes. Return hardware ERROR on
  transport failure, request base stop, and invalidate failed joint feedback.
- Send an unchanged gripper position only once; continue base heartbeat and commands.
- Publish physical finger velocity in m/s, including the DYNAMIXEL velocity scale.
- Publish current wheel positions without shared/delayed history, and wheel/arm
  velocities in rad/s as required by revolute joint state interfaces.
- Configure actual `gripper_goal_current_raw` (1..80, default 80); do not misinterpret
  GripperCommand max_effort or result.effort as measured force/current.
- Shutdown explicitly stops the base and requests torque-off without a hidden
  home/zero-gripper move or destructor delay. Torque-off is a proxy write acknowledgement,
  not independent proof that every servo is off.

`/manipulation/gripper_hardware_state` publishes checked USB transport status,
the board clock/connection flags, aggregate manipulator torque, requested and
reported finger position, raw measured/goal current, and refreshed goal readback.
The OpenCR firmware does not propagate individual servo read/write failures and
does not expose individual Torque Enable, Hardware Error Status, operating mode,
temperature, or motor voltage. A fresh proxy message cannot prove all those states.
`goal_refresh_ok` acknowledges the proxy refresh transaction; it is not a guarantee
that the firmware's internal servo read succeeded.

The internal controller can finish a GripperCommand with `stalled=true` under
allow_stalling. The pick executor still requires independent stable width feedback.
These changes do not automatically clear a motor protection shutdown or remove
physical obstruction. Those require hardware diagnosis if the stall persists.

Build/test without connecting to hardware:

```bash
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --symlink-install \
  --packages-select cleanup_interfaces manipulation_hardware
source install/setup.bash
PYTHONNOUSERSITE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test \
  --packages-select manipulation_hardware
```

The environment flags isolate the system ROS/pytest installation from incompatible
user-site pytest plugins. Tests inject a fake serial transport; they never open
the robot port. Real loaded carry-and-release has not been validated by these tests.

Firmware protocol reference:
https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp
