# Copyright 2026 TurtleBot3 Project Contributors
# Licensed under the Apache License, Version 2.0.

"""Verify launch-style process-group shutdown against fake controllers only."""

import os
import signal
import subprocess
import sys
import threading
import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState


def wait_for(predicate, timeout=5):
    """Bound fake-controller discovery and process startup."""
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(.02)
    assert predicate()


@pytest.mark.parametrize('opens', [True, False])
def test_sigint_opens_before_controller_exit(tmp_path, monkeypatch, opens):
    """Shield the hardware child from group SIGINT until opening feedback is checked."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(145 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    child = tmp_path / 'fake_controller'
    stopped = tmp_path / 'controller_stopped'
    started = tmp_path / 'controller_started'
    child.write_text('#!/usr/bin/python3\n'
                     'import signal, time\nfrom pathlib import Path\n'
                     f'Path({str(started)!r}).touch()\n'
                     'def stop(*_):\n'
                     f'    Path({str(stopped)!r}).write_text(str(time.monotonic()))\n'
                     '    raise SystemExit(0)\n'
                     'signal.signal(signal.SIGINT, stop)\n'
                     'signal.signal(signal.SIGTERM, stop)\n'
                     'while True: time.sleep(.05)\n')
    child.chmod(0o755)
    rclpy.init()
    node = rclpy.create_node('fake_shutdown_controllers')
    group = ReentrantCallbackGroup()
    joint_pub = node.create_publisher(JointState, '/joint_states', 10)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    position = [0.002]
    commanded = []

    def feedback():
        stamp = node.get_clock().now().to_msg()
        joint = JointState()
        joint.header.stamp = stamp
        joint.name, joint.position = ['gripper_left_joint'], position
        joint_pub.publish(joint)
        odom = Odometry()
        odom.header.stamp = stamp
        odom_pub.publish(odom)

    def arm(goal):
        goal.succeed()
        return FollowJointTrajectory.Result()

    def grip(goal):
        commanded.append((goal.request.command.position, time.monotonic()))
        if opens:
            position[0] = goal.request.command.position
        goal.succeed()
        return GripperCommand.Result(position=position[0], reached_goal=True)

    servers = [ActionServer(node, FollowJointTrajectory,
                            '/arm_controller/follow_joint_trajectory', arm, callback_group=group),
               ActionServer(node, GripperCommand, '/gripper_controller/gripper_cmd', grip,
                            callback_group=group)]
    node.create_timer(.02, feedback, callback_group=group)
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    output = (tmp_path / 'shutdown.log').open('w')
    process = subprocess.Popen([sys.executable, '-m', 'robot_motion.controlled_hardware',
                                '--controller-executable', str(child)], start_new_session=True,
                               stdout=output, stderr=subprocess.STDOUT)
    try:
        wait_for(lambda: started.exists() and joint_pub.get_subscription_count() > 0)
        time.sleep(.5)
        os.killpg(process.pid, signal.SIGINT)  # Exactly how launch signals its process group.
        assert process.wait(timeout=12) == 0, (tmp_path / 'shutdown.log').read_text()
        assert len(commanded) == 1 and commanded[0][0] == .019
        assert stopped.exists() and float(stopped.read_text()) > commanded[0][1]
        text = (tmp_path / 'shutdown.log').read_text()
        assert ('[SHUTDOWN_RELEASE_COMPLETE]' in text) == opens
        assert ('[SHUTDOWN_RELEASE_FAILED]' in text) != opens
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=15)
        output.close()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.shutdown()
