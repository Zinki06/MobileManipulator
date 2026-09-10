"""Exercise the real executor with quantized encoder feedback, never robot hardware."""

import math
import gc
import os
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from robot_motion.executor_node import MotionExecutor


def wait_for(predicate, seconds=5.0):
    """Wait for asynchronous ROS activity with a finite deadline."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate(), 'ROS test deadline exceeded'


@pytest.mark.parametrize('balanced', [False, True])
def test_measured_turns_safety_wait_cancel_and_exclusive_admission(
        tmp_path, monkeypatch, balanced):
    """Four quarter turns close on encoders; stopping never triggers a recenter move."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(150 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    args = []
    if balanced:
        args = ['--ros-args', '-p', 'spin_max_velocity:=0.55', '-p', 'spin_gain:=2.0',
                '-p', 'spin_acceleration:=0.8', '-p', 'rotation_settle:=0.15',
                '-p', 'reuse_stationary_readiness:=true']
    args = (args or ['--ros-args']) + ['-p', 'sync_depth_costmaps:=false']
    rclpy.init(args=args)
    motion = MotionExecutor()
    motion.wait_limit = 1.0
    fake = rclpy.create_node('fake_encoders_and_nav2')
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(motion)
    executor.add_node(fake)
    state = {'angle': 0.0, 'command': 0.0, 'block': False, 'fault': False,
             'nav_hold': False, 'ignore_cancel': False}
    events = []
    inhibits = []
    fake.create_subscription(Twist, '/cmd_vel_nav',
                             lambda msg: state.update(command=msg.angular.z), 10)
    fake.create_subscription(String, '/motion/events', lambda msg: events.append(msg.data), 30)
    fake.create_subscription(String, '/motion/inhibit',
                             lambda msg: inhibits.append(msg.data), QoSProfile(
                                 depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom_pub = fake.create_publisher(Odometry, '/odom', 10)
    status_pub = fake.create_publisher(String, '/motion_guard/status', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def tick():
        # Emulate hardware centi-unit integer conversion and final safety inhibition.
        velocity = 0.0 if state['block'] or state['fault'] else int(state['command'] * 100) / 100
        state['angle'] += velocity * 0.02
        msg = Odometry()
        msg.header.stamp = fake.get_clock().now().to_msg()
        msg.pose.pose.orientation.z = math.sin(state['angle'] / 2)
        msg.pose.pose.orientation.w = math.cos(state['angle'] / 2)
        msg.twist.twist.angular.z = velocity
        odom_pub.publish(msg)
        status_pub.publish(String(data='FAULT: map reset' if state['fault'] else
                                  'BLOCKED: collision monitor stopped motion'
                                  if state['block'] else 'READY'))

    def nav_execute(goal):
        while state['nav_hold'] and (not goal.is_cancel_requested or state['ignore_cancel']):
            time.sleep(0.02)
        if goal.is_cancel_requested:
            goal.canceled()
        else:
            goal.succeed()
        return NavigateToPose.Result()

    server = ActionServer(fake, NavigateToPose, '/navigate_to_pose', nav_execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    fake.create_timer(0.02, tick)
    spin = ActionClient(fake, Spin, '/motion/spin')
    nav = ActionClient(fake, NavigateToPose, '/motion/navigate_to_pose')
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def send(client, request):
        future = client.send_goal_async(request)
        wait_for(future.done)
        return future.result()

    try:
        assert spin.wait_for_server(timeout_sec=5)
        assert nav.wait_for_server(timeout_sec=5)
        wait_for(lambda: motion._odom is not None)
        pose = NavigateToPose.Goal()
        pose.pose.header.frame_id = 'map'
        pose.pose.pose.orientation.w = 1.0
        for index in range(4):
            start = state['angle']
            handle = send(spin, Spin.Goal(target_yaw=-math.pi / 2))
            assert handle.accepted
            result = handle.get_result_async()
            wait_for(lambda: abs(state['command']) > 0.01)
            if index == 0:
                assert not send(nav, pose).accepted
                state['block'] = True
                wait_for(lambda: any('WAITING_SAFETY' in e for e in events))
                stopped = state['angle']
                time.sleep(0.4)
                assert state['angle'] == stopped and not result.done()
                state['block'] = False
            wait_for(result.done, 10)
            assert result.result().status == GoalStatus.STATUS_SUCCEEDED
            assert abs((state['angle'] - start) + math.pi / 2) < 0.03
            wait_for(lambda: not motion._reserved)
        assert abs(state['angle'] + 2 * math.pi) < 0.12

        handle = send(spin, Spin.Goal(target_yaw=math.pi))
        result = handle.get_result_async()
        wait_for(lambda: state['command'] > 0.01)
        canceled = handle.cancel_goal_async()
        wait_for(canceled.done)
        wait_for(result.done)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        wait_for(lambda: not motion._reserved and state['command'] == 0)

        state['nav_hold'] = True
        handle = send(nav, pose)
        result = handle.get_result_async()
        wait_for(lambda: any('NAVIGATING' in e for e in events))
        canceled = handle.cancel_goal_async()
        wait_for(canceled.done)
        wait_for(result.done)
        assert result.result().status == GoalStatus.STATUS_CANCELED
        assert not motion._fault
        wait_for(lambda: not motion._reserved)
        state['nav_hold'] = False
        handle = send(nav, pose)
        result = handle.get_result_async()
        wait_for(result.done)
        assert result.result().status == GoalStatus.STATUS_SUCCEEDED
        wait_for(lambda: not motion._reserved)

        handle = send(spin, Spin.Goal(target_yaw=math.pi))
        result = handle.get_result_async()
        wait_for(lambda: state['command'] > 0.01)
        state['block'] = True
        wait_for(result.done)
        assert result.result().status == GoalStatus.STATUS_ABORTED
        wait_for(lambda: state['command'] == 0)
        assert any('safety wait expired' in e for e in events)
        wait_for(lambda: not motion._reserved)
        state['block'] = False
        state['nav_hold'] = state['ignore_cancel'] = True
        events.clear()
        handle = send(nav, pose)
        result = handle.get_result_async()
        wait_for(lambda: any('NAVIGATING' in e for e in events))
        canceled = handle.cancel_goal_async()
        wait_for(canceled.done)
        wait_for(result.done, 6)
        assert motion._fault and 'cancellation unconfirmed' in motion._fault
        wait_for(lambda: bool(inhibits))
        assert not send(spin, Spin.Goal(target_yaw=0.2)).accepted
    finally:
        state['nav_hold'] = False
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        server.destroy()
        spin.destroy()
        nav.destroy()
        fake.destroy_node()
        motion.destroy_node()
        gc.collect()
        rclpy.shutdown()
