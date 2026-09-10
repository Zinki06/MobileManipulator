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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String
from rclpy.qos import DurabilityPolicy, QoSProfile
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
    'candidate_holding', 'candidate_empty', 'candidate_false_open', 'candidate_relaxed',
    'candidate_processed',
    'optimized_park', 'interrupt', 'stop_holding', 'cancel_stuck', 'late_accept', 'stop_place',
    'place_delayed_open', 'place_stalled', 'place_stale', 'place_abort', 'place_hold_stop',
])
def test_strict_reachability_insertion_and_observation_pose(tmp_path, monkeypatch, outcome):
    """Reject shallow targets and execute insertion before closing at a closer target."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(130 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    rclpy.init()
    node = rclpy.create_node('fake_arm_controllers')
    group = ReentrantCallbackGroup()
    paths, grippers, phases, carrying = [], [], [], []
    candidate_mode = outcome.startswith('candidate_')
    behavior = ('holding' if outcome in {
        'candidate_relaxed', 'candidate_processed', 'interrupt', 'stop_holding',
        'cancel_stuck', 'late_accept', 'stop_place',
        'place_delayed_open', 'place_stalled', 'place_stale', 'place_abort', 'place_hold_stop'}
                else outcome.removeprefix('candidate_'))
    finger = [0.019]
    arm_position = [0.0, -0.523, -0.523, 1.5707]
    reject_lowering = [False]
    block_arm = [False]
    arm_waiting = threading.Event()
    placement_active = [False]
    open_attempts = []
    placement_close_done = [None]
    stale_feedback = [False]
    joint_pub = node.create_publisher(JointState, '/joint_states', 10)

    def feedback():
        if stale_feedback[0]:
            return
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = ['gripper_left_joint', 'joint1', 'joint2', 'joint3', 'joint4']
        msg.position = [finger[0]] + arm_position
        msg.velocity = [0.] * 5
        joint_pub.publish(msg)

    node.create_timer(0.02, feedback, callback_group=group)

    def arm(goal):
        paths.append(goal.request.trajectory)
        if block_arm[0]:
            arm_waiting.set()
            deadline = time.monotonic() + 5
            while (not goal.is_cancel_requested or outcome == 'cancel_stuck') and \
                    time.monotonic() < deadline:
                time.sleep(.02)
            if goal.is_cancel_requested:
                goal.canceled()
                return FollowJointTrajectory.Result(error_code=-1)
            goal.abort()
            return FollowJointTrajectory.Result(error_code=-1)
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
        if placement_active[0] and goal.request.command.position > 0:
            assert placement_close_done[0] is not None
            assert time.monotonic() - placement_close_done[0] >= 5.0
            open_attempts.append((goal.request.command.position, goal.request.command.max_effort))
            if outcome == 'place_abort':
                goal.abort()
                return GripperCommand.Result(position=finger[0], reached_goal=False)
            if outcome == 'place_delayed_open':
                time.sleep(3.5)  # Longer than the former three-second action deadline.
            if outcome == 'place_stalled':
                goal.succeed()
                return GripperCommand.Result(position=finger[0], stalled=True, reached_goal=False)
            if outcome == 'place_stale':
                stale_feedback[0] = True
                goal.succeed()
                return GripperCommand.Result(position=.019, reached_goal=True)
        if goal.request.command.position < 0 and outcome in {'stop_holding', 'cancel_stuck'}:
            block_arm[0] = True
        finger[0] = (0.009 if behavior == 'false_open' else 0.019) \
            if goal.request.command.position > 0 else \
            (-0.00996 if behavior == 'empty' else 0.002)
        if placement_active[0] and goal.request.command.position < 0:
            placement_close_done[0] = time.monotonic()
        goal.succeed()
        return GripperCommand.Result(position=finger[0], reached_goal=True)

    def accept_arm(_request):
        if outcome == 'late_accept' and block_arm[0]:
            time.sleep(3.6)
        return GoalResponse.ACCEPT

    arm_server = ActionServer(node, FollowJointTrajectory,
                              '/arm_controller/follow_joint_trajectory', arm,
                              goal_callback=accept_arm,
                              cancel_callback=lambda _: CancelResponse.ACCEPT,
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
    node.create_subscription(Bool, '/cleanup/carrying', lambda msg: carrying.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
    if outcome in {'candidate_relaxed', 'candidate_processed'}:
        args += ['-p', 'grasp_min_body_depth:=0.004', '-p', 'approach_error_margin:=0.0']
    if outcome == 'optimized_park':
        args += ['--ros-args', '-p', 'startup_park:=false',
                 '-p', 'reached_joint_tolerance:=0.015']
    process = subprocess.Popen(args, stdout=output, stderr=subprocess.STDOUT)

    def trigger(name):
        client = node.create_client(Trigger, name)
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(Trigger.Request())
        wait_for(future.done, 20)
        return future.result()

    try:
        if outcome == 'optimized_park':
            client = node.create_client(Trigger, '/park_arm')
            assert client.wait_for_service(timeout_sec=5)
            time.sleep(.3)
            assert not paths
            assert trigger('/park_arm').success
            assert not paths  # Fresh stationary feedback avoids a redundant 3 s action.
            assert trigger('/observe_floor').success
            assert len(paths) == 1
            time.sleep(.1)
            assert trigger('/park_arm').success
            assert len(paths) == 2  # A different pose must still execute.
            return
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
        if outcome == 'candidate_relaxed':
            req.target.point.x, req.target.point.z = 0.5, 0.029
            checked = evaluate.call_async(req)
            wait_for(checked.done)
            response = checked.result()
            assert response.success and not response.reachable
            assert response.approach_available
            assert 'min_body_depth=0.004000' in response.message
            assert 'approach_error_margin=0.000000' in response.message
            assert not grippers  # A proposed approach never executes the grasp.
            # Disabling the hypothetical error margin admits the nominal 22cm pose.
            req.target.point.z = 0.0324
            checked = evaluate.call_async(req)
            wait_for(checked.done)
            assert checked.result().approach_available
            assert abs(checked.result().approach_pose.pose.position.x - 0.28) < 1e-6
            # An arrival inside Nav2's 2cm tolerance can still be outside arm reach.
            # Retry the next closer pose, not the same already-reached goal.
            req.target.point.x, req.target.point.z = 0.22, 0.029
            checked = evaluate.call_async(req)
            wait_for(checked.done)
            assert not checked.result().reachable and checked.result().approach_available
            assert abs(checked.result().approach_pose.pose.position.x - 0.04) < 1e-6
        if outcome == 'candidate_holding':
            req.target.point.x = 0.5
            for height in (0.0213585, 0.0231518):
                req.target.point.z = height
                rejected = evaluate.call_async(req)
                wait_for(rejected.done)
                response = rejected.result()
                assert response.success and not response.reachable
                assert not response.approach_available
                assert 'insufficient_body_depth=27' in response.message
                assert 'hover_unreachable=0' in response.message
            req.target.point.z = 0.0324
            corrected = evaluate.call_async(req)
            wait_for(corrected.done)
            assert corrected.result().approach_available
            assert abs(corrected.result().approach_pose.pose.position.x - 0.30) < 1e-6
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
        if outcome == 'candidate_processed':
            from rclpy.duration import Duration

            # Recorded successful reachability target from run 200254, in the fake base frame.
            req.target.point.x = 0.215559
            req.target.point.y = 0.031291
            req.target.point.z = 0.032088
            for delay in (7.0, -2.0):
                stamp = node.get_clock().now() - Duration(seconds=delay)
                req.target.header.stamp = stamp.to_msg()
                target_pub.publish(req.target)
                time.sleep(0.1)
                rejected = trigger('/execute_pick_and_place')
                assert not rejected.success and rejected.message.startswith('TARGET_EXPIRED:')
                assert not grippers
            req.target.header.stamp = (node.get_clock().now() - Duration(seconds=2.07)).to_msg()
        target_pub.publish(req.target)
        time.sleep(0.1)
        paths.clear()
        if outcome in {'stop_holding', 'cancel_stuck', 'late_accept'}:
            block_arm[0] = outcome == 'late_accept'
            client = node.create_client(Trigger, '/execute_pick_and_place')
            assert client.wait_for_service(timeout_sec=3)
            future = client.call_async(Trigger.Request())
            if outcome == 'late_accept':
                wait_for(future.done, 8)
                wait_for(arm_waiting.is_set, 8)
                time.sleep(.5)
                assert not future.result().success
            else:
                wait_for(arm_waiting.is_set, 8)
                stop = node.create_client(Trigger, '/cleanup/stop_manipulation')
                assert stop.wait_for_service(timeout_sec=3)
                stopped = stop.call_async(Trigger.Request())
                wait_for(stopped.done, 10)
                assert stopped.result().success == (outcome == 'stop_holding')
                wait_for(future.done, 10)
                assert finger[0] == .002
            assert grippers.count(.019) == 1  # Stop never opens the held object.
            if outcome != 'stop_holding':
                assert any(p.startswith('ACTION_FAULT|') for p in phases)
                count = len(paths)
                assert not trigger('/park_arm').success
                assert not trigger('/open_gripper').success
                assert len(paths) == count
            return
        if outcome == 'interrupt':
            block_arm[0] = True
            client = node.create_client(Trigger, '/execute_pick_and_place')
            assert client.wait_for_service(timeout_sec=3)
            future = client.call_async(Trigger.Request())
            wait_for(arm_waiting.is_set)
            assert trigger('/open_gripper').success
            wait_for(future.done)
            assert not future.result().success
            time.sleep(.3)
            assert grippers == [.019, .019]  # No late CLOSE after the terminal opening.
            assert not any(carrying)
            return
        result = trigger('/execute_pick_and_place')
        if behavior != 'holding':
            assert not result.success
            if behavior == 'false_open':
                assert not paths  # Never descend with an incompletely opened hand.
            else:
                assert result.message.startswith('EMPTY_GRASP:')
            assert not any(carrying)
            assert not any(p.startswith('SEQUENCE_COMPLETE|') for p in phases)
            return
        assert result.success
        wait_for(lambda: carrying and carrying[-1])
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
            assert f'target_link1={req.target.point.x + 0.122:.6f}' in plan_phase
            if outcome == 'candidate_processed':
                assert any(p.startswith('TARGET_ACCEPTED|') and 'limit=6.000000s' in p
                           for p in phases)
            assert paths[2].points[-1].positions == paths[0].points[-1].positions
            assert -30 < -sum(paths[1].points[-1].positions[1:]) * 180 / math.pi < -15
            assert trigger('/open_gripper').success
            wait_for(lambda: carrying and not carrying[-1])
            assert grippers[-1] == 0.019
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
        if outcome == 'stop_place':
            block_arm[0] = True
            arm_waiting.clear()
            future = place.call_async(placement)
            wait_for(arm_waiting.is_set)
            assert trigger('/cleanup/stop_manipulation').success
            wait_for(future.done)
            assert not future.result().success and not future.result().released
            assert len(grippers) == opened_before
            assert finger[0] == .002
            return
        reject_lowering[0] = True
        future = place.call_async(placement)
        wait_for(future.done, 20)
        assert not future.result().success and not future.result().released
        assert len(grippers) == opened_before
        reject_lowering[0] = False
        placement.floor_target.header.stamp = node.get_clock().now().to_msg()
        placement_active[0] = True
        paths_before = len(paths)
        future = place.call_async(placement)
        if outcome == 'place_hold_stop':
            wait_for(lambda: any(p.startswith('PLACE_HOLD|') for p in phases), 15)
            assert trigger('/cleanup/stop_manipulation').success
        wait_for(future.done, 20)
        if outcome in {'place_stalled', 'place_stale', 'place_abort', 'place_hold_stop'}:
            assert not future.result().success and not future.result().released
            attempts = 0 if outcome == 'place_hold_stop' else 1
            assert open_attempts == [(.019, 10.)] * attempts
            assert len(paths) == paths_before + 2  # Approach and lower only, no retreat.
            assert not any(p.startswith('PLACE_RETREAT|') for p in phases)
            assert finger[0] == .002
            return
        assert future.result().success and future.result().released
        assert open_attempts == [(.019, 10.)]
        assert len(paths) == paths_before + 3  # Approach, lower, then retreat only after release.
        tags = [p.split('|')[0] for p in phases]
        assert tags.index('PLACE_LOWER') < tags.index('PLACE_OPEN') < tags.index('PLACE_RETREAT')
        assert grippers[-2:] == [-0.010, 0.019]
        assert tags.index('PLACE_LOWER') < tags.index('PLACE_RECLOSE') < (
            tags.index('PLACE_HOLD')) < tags.index('PLACE_OPEN')
        wait_for(lambda: carrying and not carrying[-1])
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
