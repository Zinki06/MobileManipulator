# Copyright 2026 TurtleBot3 Project Contributors
# Licensed under the Apache License, Version 2.0.

"""Keep ros2_control alive long enough to open the gripper on launch shutdown."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_prefix
from control_msgs.action import GripperCommand
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class HardwareShutdown(Node):
    """Cancel manipulation and verify finger opening before controller teardown."""

    def __init__(self):
        """Connect to the same controllers used by the pick executor."""
        super().__init__('hardware_shutdown', use_global_arguments=False)
        self.gripper = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.cancels = [self.create_client(CancelGoal, name + '/_action/cancel_goal') for name in (
            '/arm_controller/follow_joint_trajectory', '/gripper_controller/gripper_cmd')]
        self.position = None
        self.joint_stamp = None
        self.stopped_since = None
        self.odom_received = 0.0
        self.create_subscription(JointState, '/joint_states', self._joints,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data)
        self.inhibit = self.create_publisher(String, '/motion/inhibit', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _joints(self, msg):
        for index, name in enumerate(msg.name):
            if name == 'gripper_left_joint' and index < len(msg.position):
                self.position = msg.position[index]
                self.joint_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

    def _odom(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age < 0.0 or age > 0.25:
            self.stopped_since = None
            return
        self.odom_received = time.monotonic()
        speed = msg.twist.twist
        if abs(speed.linear.x) < 0.01 and abs(speed.angular.z) < 0.02:
            if self.stopped_since is None:
                self.stopped_since = self.odom_received
        else:
            self.stopped_since = None

    def _wait(self, predicate, seconds):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        return predicate()

    def release(self):
        """Bound shutdown work and report whether fresh encoders confirm opening."""
        self.get_logger().info('[SHUTDOWN_RELEASE_START] Keeping controllers alive')
        self.inhibit.publish(String(data='Hardware shutdown: stop before finger release'))
        stopped = self._wait(lambda: self.stopped_since is not None and
                             time.monotonic() - self.stopped_since >= 0.3 and
                             time.monotonic() - self.odom_received < 0.25, 1.5)
        if not stopped:
            self.get_logger().error('[SHUTDOWN_RELEASE_FAILED] No stationary base feedback')
            return False
        for client in self.cancels:
            if not client.wait_for_service(timeout_sec=0.5):
                self.get_logger().error('[SHUTDOWN_RELEASE_FAILED] Controller unavailable')
                return False
            future = client.call_async(CancelGoal.Request())
            if not self._wait(future.done, 1.0) or future.result() is None or \
                    future.result().return_code != CancelGoal.Response.ERROR_NONE:
                self.get_logger().error('[SHUTDOWN_RELEASE_FAILED] Cancellation timeout')
                return False
        if not self.gripper.wait_for_server(timeout_sec=0.5):
            self.get_logger().error('[SHUTDOWN_RELEASE_FAILED] Gripper unavailable')
            return False
        goal = GripperCommand.Goal()
        goal.command.position = 0.019
        goal.command.max_effort = 10.0
        future = self.gripper.send_goal_async(goal)
        if not self._wait(future.done, 1.0) or not future.result().accepted:
            self.get_logger().error('[SHUTDOWN_RELEASE_FAILED] Opening goal rejected')
            return False
        result = future.result().get_result_async()
        success = self._wait(result.done, 3.0) and \
            result.result().status == GoalStatus.STATUS_SUCCEEDED
        success = success and self._wait(lambda: self.joint_stamp is not None and
                                         0.0 <= (self.get_clock().now() -
                                                 self.joint_stamp).nanoseconds / 1e9 < 0.25 and
                                         abs(self.position - 0.019) <= 0.0015, 1.0)
        if success:
            self.get_logger().info('[SHUTDOWN_RELEASE_COMPLETE] Fingers open: 0.019 m')
        else:
            self.get_logger().error('[SHUTDOWN_RELEASE_FAILED] Opening not confirmed')
        return success

    def destroy_node(self):
        """Destroy action waitables before the ROS context shuts down."""
        self.gripper.destroy()
        return super().destroy_node()


def main(args=None):
    """Forward controller arguments and defer its termination until release completes."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--controller-executable', default=str(
        Path(get_package_prefix('controller_manager')) /
        'lib/controller_manager/ros2_control_node'))
    options, controller_args = parser.parse_known_args(args)
    stopping = False

    def stop(_number, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = HardwareShutdown()
    # Launch signals the wrapper process group; the controller needs its own group.
    child = subprocess.Popen([options.controller_executable, *controller_args],
                             start_new_session=True)
    try:
        while child.poll() is None and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
        if stopping and child.poll() is None:
            try:
                node.release()
            except Exception as error:
                node.get_logger().error(f'[SHUTDOWN_RELEASE_FAILED] {error}')
    finally:
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        node.destroy_node()
        rclpy.shutdown()
    if not stopping and child.returncode:
        raise SystemExit(child.returncode)


if __name__ == '__main__':
    main()
