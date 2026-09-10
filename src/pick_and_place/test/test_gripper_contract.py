# Copyright 2026 TurtleBot3 Project Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise gripper preflight and typed results on an isolated fake ROS domain."""

import os
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from cleanup_interfaces.msg import GripperHardwareState
from cleanup_interfaces.srv import ExecutePick
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PointStamped, TransformStamped
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


@pytest.mark.parametrize('failure', ['preflight', 'open', 'stale_close', 'transport', 'none'])
def test_gripper_result_and_health(tmp_path, monkeypatch, failure):
    """Do not confuse pre-close failure, unknown feedback, or real possession."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(145 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    rclpy.init()
    node = rclpy.create_node('gripper_contract_fixture')
    group = ReentrantCallbackGroup()
    finger, arm = [.019], [0., -.523, -.523, 1.5707]
    goals, paths = [], []
    feedback_enabled, transport_ok = [True], [True]
    carrying = [False]
    node.create_subscription(Bool, '/cleanup/carrying',
                             lambda msg: carrying.__setitem__(0, msg.data), 10)
    joints = node.create_publisher(JointState, '/joint_states', 10)
    health = node.create_publisher(
        GripperHardwareState, '/manipulation/gripper_hardware_state',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def publish():
        stamp = node.get_clock().now().to_msg()
        if feedback_enabled[0]:
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = ['gripper_left_joint', 'joint1', 'joint2', 'joint3', 'joint4']
            msg.position, msg.velocity = finger + arm, [0.] * 5
            joints.publish(msg)
        state = GripperHardwareState()
        state.header.stamp = stamp
        state.read_ok = state.command_ok = transport_ok[0]
        state.ros_connected = state.manipulator_connected = True
        state.all_joints_torque_enabled = True
        state.position = finger[0]
        state.fault = '' if transport_ok[0] else 'fixture commit failure'
        health.publish(state)

    node.create_timer(.02, publish)

    def move_arm(handle):
        paths.append(handle.request.trajectory)
        arm[:] = handle.request.trajectory.points[-1].positions
        handle.succeed()
        return FollowJointTrajectory.Result()

    def grip(handle):
        target = handle.request.command.position
        goals.append(target)
        stalled = target > 0 and (
            failure == 'preflight' or failure == 'open' and len(goals) > 1)
        finger[0] = -.0019 if stalled else .019 if target > 0 else .002
        if target < 0 and failure == 'stale_close':
            feedback_enabled[0] = False
        if target < 0 and failure == 'transport':
            transport_ok[0] = False
        handle.succeed()
        return GripperCommand.Result(position=finger[0], stalled=stalled,
                                     reached_goal=not stalled)

    servers = [ActionServer(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
                            move_arm, callback_group=group),
               ActionServer(node, GripperCommand, '/gripper_controller/gripper_cmd',
                            grip, callback_group=group)]
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'map', 'link1'
    tf.transform.translation.x, tf.transform.translation.z = -.092, .101
    tf.transform.rotation.w = 1.
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(tf)
    targets = node.create_publisher(PointStamped, '/object_centroid', 10)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    output = (tmp_path / 'pick.log').open('w')
    executable = Path(get_package_prefix('pick_and_place')) / 'lib/pick_and_place/pick_and_place'
    process = None

    def call(kind, name):
        client = node.create_client(kind, name)
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(kind.Request())
        deadline = time.monotonic() + 15
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.02)
        assert future.done(), (tmp_path / 'pick.log').read_text()
        return future.result()

    try:
        process = subprocess.Popen([str(executable), '--ros-args', '-p', 'startup_park:=false',
                                    '-p', 'require_gripper_hardware_state:=true'],
                                   stdout=output, stderr=subprocess.STDOUT)
        ready = node.create_client(Trigger, '/cleanup/prepare_gripper')
        assert ready.wait_for_service(timeout_sec=5)
        time.sleep(.4)
        prepared = call(Trigger, '/cleanup/prepare_gripper')
        assert prepared.success == (failure != 'preflight')
        if failure == 'preflight':
            assert not paths and goals == [.019]
            assert 'GRIPPER_FAULT' in prepared.message
            return
        target = PointStamped()
        target.header.frame_id = 'map'
        target.header.stamp = node.get_clock().now().to_msg()
        target.point.x, target.point.z = .20, .051
        targets.publish(target)
        time.sleep(.1)
        result = call(ExecutePick, '/cleanup/execute_pick')
        if failure == 'open':
            assert not result.success and result.result_code == result.GRIPPER_FAILED
            assert result.stage == 'OPEN' and not result.close_started
            assert result.holding_state == result.HOLD_EMPTY
            assert not paths
        elif failure in {'stale_close', 'transport'}:
            assert not result.success and result.result_code == result.GRIPPER_FAILED
            assert result.stage == 'CLOSE' and result.close_started
            assert result.holding_state == result.HOLD_UNKNOWN
            assert 'EMPTY_GRASP:' not in result.message
        else:
            assert result.success and result.result_code == result.OK
            assert result.holding_state == result.HOLD_CONFIRMED and result.close_started
            assert not call(Trigger, '/cleanup/prepare_gripper').success
            deadline = time.monotonic() + 2
            while not carrying[0] and time.monotonic() < deadline:
                time.sleep(.02)
            assert carrying[0]
            transport_ok[0] = False
            deadline = time.monotonic() + 2
            while carrying[0] and time.monotonic() < deadline:
                time.sleep(.02)
            assert not carrying[0], 'Fresh joint positions cannot hide failed hardware transport'
    finally:
        if process is not None:
            process.terminate()
            process.wait(timeout=10)
        output.close()
        executor.shutdown()
        thread.join(timeout=5)
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.shutdown()
