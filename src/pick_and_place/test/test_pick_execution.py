"""Exercise the actual arm node against fake controllers, never real hardware."""

import math
import os
from pathlib import Path
import subprocess
import threading
import time
import pytest

from ament_index_python.packages import get_package_prefix
from cleanup_interfaces.srv import EvaluateGrasp, PlaceObject
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PointStamped, TransformStamped
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster


def wait_for(predicate, seconds=5):
    """Bound ROS discovery and action completion waits."""
    deadline = time.monotonic() + seconds
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert predicate(), 'ROS fake-controller timeout'


@pytest.mark.parametrize('outcome', [
    'holding', 'empty', 'drop', 'false_open',
    'candidate_holding', 'candidate_empty', 'candidate_false_open',
])
def test_strict_reachability_insertion_and_observation_pose(tmp_path, monkeypatch, outcome):
    """Reject shallow targets and execute insertion before closing at a closer target."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(130 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    rclpy.init()
    node = rclpy.create_node('fake_arm_controllers')
    group = ReentrantCallbackGroup()
    paths, grippers, phases = [], [], []
    candidate_mode = outcome.startswith('candidate_')
    behavior = outcome.removeprefix('candidate_')
    finger = [0.019]
    arm_position = [0.0, -0.523, -0.523, 1.5707]
    reject_lowering = [False]
    joint_pub = node.create_publisher(JointState, '/joint_states', 10)

    def feedback():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = ['gripper_left_joint', 'joint1', 'joint2', 'joint3', 'joint4']
        msg.position = [finger[0]] + arm_position
        joint_pub.publish(msg)

    node.create_timer(0.02, feedback, callback_group=group)

    def arm(goal):
        paths.append(goal.request.trajectory)
        if reject_lowering[0] and len(goal.request.trajectory.points) == 10:
            goal.abort()
            return FollowJointTrajectory.Result(error_code=-1)
        arm_position[:] = goal.request.trajectory.points[-1].positions
        if behavior == 'drop' and len(paths) == 4:
            finger[0] = -0.00996
        goal.succeed()
        return FollowJointTrajectory.Result()

    def gripper(goal):
        grippers.append(goal.request.command.position)
        finger[0] = (0.009 if behavior == 'false_open' else 0.019) \
            if goal.request.command.position > 0 else \
            (-0.00996 if behavior == 'empty' else 0.002)
        goal.succeed()
        return GripperCommand.Result(position=finger[0], reached_goal=True)

    arm_server = ActionServer(node, FollowJointTrajectory,
                              '/arm_controller/follow_joint_trajectory', arm,
                              callback_group=group)
    grip_server = ActionServer(node, GripperCommand, '/gripper_controller/gripper_cmd',
                               gripper, callback_group=group)
    transforms = []
    for parent, child, x, z in [
            ('map', 'base_link', 0., 0.), ('base_link', 'link1', -0.092, 0.101)]:
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = parent, child
        tf.header.stamp = node.get_clock().now().to_msg()
        tf.transform.translation.x, tf.transform.translation.z = x, z
        tf.transform.rotation.w = 1.0
        transforms.append(tf)
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(transforms)
    target_pub = node.create_publisher(PointStamped, '/object_centroid', 10)
    node.create_subscription(String, '/pick/events', lambda msg: phases.append(msg.data), 20)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    executable = Path(get_package_prefix('pick_and_place')) / 'lib/pick_and_place/pick_and_place'
    output = (tmp_path / 'arm.log').open('w')
    args = [str(executable)]
    if candidate_mode:
        args += ['--ros-args', '-p', 'use_candidate_grasp:=true',
                 '-p', 'grasp_forward_offset:=0.03']
    process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)

    def trigger(name):
        client = node.create_client(Trigger, name)
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(Trigger.Request())
        wait_for(future.done, 20)
        return future.result()

    try:
        wait_for(lambda: bool(paths), 10)  # Startup pose, no physical actuators connected.
        time.sleep(0.3)
        assert trigger('/observe_floor').success
        assert abs(paths[-1].points[-1].positions[3] - 1.7707) < 1e-5
        assert trigger('/park_arm').success
        assert abs(paths[-1].points[-1].positions[3] - 1.5707) < 1e-5
        evaluate = node.create_client(EvaluateGrasp, '/cleanup/evaluate_grasp')
        assert evaluate.wait_for_service(timeout_sec=3)
        req = EvaluateGrasp.Request()
        req.target.header.frame_id = 'map'
        req.require_candidate = candidate_mode
        req.target.point.x, req.target.point.z = (0.5, 0.034) if candidate_mode else (0.255, 0.051)
        future = evaluate.call_async(req)
        wait_for(future.done)
        assert future.result().success and not future.result().reachable
        assert future.result().approach_available
        req.target.header.stamp = node.get_clock().now().to_msg()
        target_pub.publish(req.target)
        time.sleep(0.1)
        assert not trigger('/execute_pick_and_place').success
        assert not grippers

        if candidate_mode:
            assert trigger('/observe_floor').success
        req.target.point.x = 0.21 if candidate_mode else 0.20
        req.target.header.stamp = node.get_clock().now().to_msg()
        target_pub.publish(req.target)
        time.sleep(0.1)
        paths.clear()
        result = trigger('/execute_pick_and_place')
        if behavior != 'holding':
            assert not result.success
            if behavior == 'false_open':
                assert not paths  # Never descend with an incompletely opened hand.
            else:
                assert result.message.startswith('EMPTY_GRASP:')
            assert not any(p.startswith('SEQUENCE_COMPLETE|') for p in phases)
            return
        assert result.success
        wait_for(lambda: any(p.startswith('SEQUENCE_COMPLETE|') for p in phases))
        tags = [p.split('|')[0] for p in phases]
        if candidate_mode:
            assert 'INSERT' not in tags
            assert tags.index('DESCEND') < tags.index('CLOSE') < tags.index('LIFT')
            assert [len(p.points) for p in paths] == [1, 10, 10, 1]
            plan_phase = next(p for p in phases if p.startswith('PLAN|'))
            assert 'strategy=candidate_body' in plan_phase
            assert 'forward_offset=0.030000' in plan_phase
            # Observed map x=.21 plus arm offset .092 plus correction .03 exactly once.
            assert 'target_link1=0.332000' in plan_phase
            assert paths[2].points[-1].positions == paths[0].points[-1].positions
            assert -30 < -sum(paths[1].points[-1].positions[1:]) * 180 / math.pi < -15
            return
        assert tags.index('DESCEND') < tags.index('INSERT') < tags.index('CLOSE')
        assert [len(p.points) for p in paths] == [1, 10, 4, 10, 1]
        for trajectory in paths[1:4]:
            times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                     for p in trajectory.points]
            assert all(b > a for a, b in zip([0] + times, times))
            for point in trajectory.points:
                pitch = -sum(point.positions[1:]) * 180 / math.pi
                assert -65.01 <= pitch <= -54.99
        assert grippers == [0.019, -0.01]
        place = node.create_client(PlaceObject, '/cleanup/place_object')
        assert place.wait_for_service(timeout_sec=3)
        placement = PlaceObject.Request()
        placement.floor_target.header.frame_id = 'map'
        placement.floor_target.header.stamp = node.get_clock().now().to_msg()
        placement.floor_target.point.x = 0.6
        placement.release_height = 0.035
        opened_before = len(grippers)
        future = place.call_async(placement)
        wait_for(future.done, 10)
        assert not future.result().success and not future.result().released
        assert len(grippers) == opened_before
        placement.floor_target.point.x = 0.24
        placement.floor_target.header.stamp = node.get_clock().now().to_msg()
        reject_lowering[0] = True
        future = place.call_async(placement)
        wait_for(future.done, 20)
        assert not future.result().success and not future.result().released
        assert len(grippers) == opened_before
        reject_lowering[0] = False
        placement.floor_target.header.stamp = node.get_clock().now().to_msg()
        future = place.call_async(placement)
        wait_for(future.done, 20)
        assert future.result().success and future.result().released
        tags = [p.split('|')[0] for p in phases]
        assert tags.index('PLACE_LOWER') < tags.index('PLACE_OPEN') < tags.index('PLACE_RETREAT')
        assert grippers[-1] == 0.019
    finally:
        process.terminate()
        process.wait(timeout=10)
        output.close()
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        arm_server.destroy()
        grip_server.destroy()
        node.destroy_node()
        rclpy.shutdown()
