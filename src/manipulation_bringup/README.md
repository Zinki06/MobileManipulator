# Manipulation bringup

`ros2 launch manipulation_bringup hardware.launch.py` starts the independent
OpenCR driver, standard wheel/arm/gripper controllers, and robot state publisher.
The project launch includes this package. This command activates real motors;
unit tests only evaluate launch descriptions and fake transports.

The existing description URDF/meshes are reused read-only. The expanded in-memory
model selects `manipulation_hardware/OpenCRSystem`; the old hardware plugin and
bringup configuration are not used. `use_fake_hardware:=true` retains GenericSystem.
`use_sim:=true` is rejected because this launch does not start a simulator.

The controller loop is 50 Hz to match the observed shared serial transport rate.
JointState and action endpoint names remain unchanged. The hardware shutdown
wrapper remains in robot_motion and opens before controller teardown.

The cleanup manager parks the arm and calls `/cleanup/prepare_gripper` before its
first navigation. `/cleanup/execute_pick` returns phase, result code and possession
state. The old `/execute_pick_and_place` Trigger remains for manual tools.
