"""Exercise the installed C++ depth publisher on localhost-only fake camera data."""

import os
from pathlib import Path
import subprocess
import threading
import time
from types import SimpleNamespace

from ament_index_python.packages import get_package_prefix
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster

from cleanup_perception.obstacle_depth_node import depth_points


def test_cpp_cloud_contract_and_stale_input(tmp_path, monkeypatch):
    """Preserve XYZ/header semantics for both depth formats and reject stale input."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(180 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    executable = Path(get_package_prefix('aruco_localizer')) / \
        'lib/aruco_localizer/obstacle_depth_node'
    output = (tmp_path / 'depth.log').open('w')
    process = subprocess.Popen([str(executable)], stdout=output, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('fake_depth_camera')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    received = []
    node.create_subscription(PointCloud2, '/cleanup/obstacle_points',
                             lambda msg: received.append((time.monotonic(), msg)), 1)
    info_pub = node.create_publisher(CameraInfo, '/camera/camera/color/camera_info', 1)
    image_pub = node.create_publisher(Image, '/camera/camera/aligned_depth_to_color/image_raw', 1)
    info = CameraInfo(width=128, height=128, k=[100., 0., 64., 0., 100., 64., 0., 0., 1.])
    carry_pub = node.create_publisher(
        Bool, '/cleanup/carrying',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    joint_pub = node.create_publisher(JointState, '/joint_states', 1)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_color_optical_frame'
    transform.transform.translation.x = 0.12
    transform.transform.translation.z = 0.03
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    state = {'carry': None, 'parked': True, 'joint_stale': False, 'encoding': '16UC1', 'stale': False, 'invalid': False}
    values = np.full((64, 64), 1000, dtype=np.uint16)
    values[0, 0] = 0
    values[8, 8] = 4000

    def publish():
        info_pub.publish(info)
        message = Image(height=64, width=64, encoding=state['encoding'], is_bigendian=1)
        message.header.frame_id = 'camera_color_optical_frame'
        now = node.get_clock().now().nanoseconds - (1000000000 if state['stale'] else 0)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(now, 1000000000)
        data = np.zeros_like(values) if state['invalid'] else values
        data = data.astype('>u2') if state['encoding'] == '16UC1' else \
            (data.astype(np.float32) / 1000).astype('>f4')
        message.step = 64 * data.dtype.itemsize + 4
        message.data = b''.join(row.tobytes() + bytes(4) for row in data)
        image_pub.publish(message)
        if state['carry'] is not None:
            carry_pub.publish(Bool(data=state['carry']))
        joint = JointState()
        joint.header.stamp = node.get_clock().now().to_msg()
        if state['joint_stale']:
            joint.header.stamp.sec -= 1
        joint.name = ['joint1', 'joint2', 'joint3', 'joint4']
        joint.position = [0., -0.523, -0.523, 1.5707 if state['parked'] else 0.5]
        joint.velocity = [0.] * 4
        joint_pub.publish(joint)

    node.create_timer(1 / 30, publish)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 5
        while len(received) < 3 and time.monotonic() < deadline:
            time.sleep(.02)
        assert len(received) >= 3, (tmp_path / 'depth.log').read_text()
        reference = depth_points(values, SimpleNamespace(k=info.k, width=128, height=128))
        for encoding in ('16UC1', '32FC1'):
            state['encoding'] = encoding
            time.sleep(.3)
            cloud = received[-1][1]
            actual = np.frombuffer(cloud.data, dtype='>f4' if cloud.is_bigendian else '<f4')
            np.testing.assert_allclose(actual.reshape(-1, 3), reference, rtol=1e-6, atol=1e-6)
            assert cloud.header.frame_id == 'camera_color_optical_frame'
            assert [f.name for f in cloud.fields] == ['x', 'y', 'z']
            assert cloud.point_step == 12
        start = time.monotonic()
        time.sleep(2)
        rate = sum(t >= start for t, _ in received) / (time.monotonic() - start)
        assert 6 <= rate <= 11
        for key in ('stale', 'invalid'):
            state[key] = True
            time.sleep(.3)
            count = len(received)
            time.sleep(.5)
            assert len(received) == count
            state[key] = False
        info.k = [1000., 0., 64., 0., 1000., 64., 0., 0., 1.]
        values[:] = 200
        values[0, 0] = 1000  # Real obstacles outside the carried-object envelope.
        values[8, 8] = 1000
        state['carry'] = True
        time.sleep(.6)
        assert received[-1][1].width == 2
        remaining = np.frombuffer(received[-1][1].data, dtype='>f4' if
                                  received[-1][1].is_bigendian else '<f4').reshape(-1, 3)
        np.testing.assert_allclose(remaining[:, 2], [1., 1.])
        values[:] = 200
        time.sleep(.3)
        assert received[-1][1].width == 0  # Valid depth consisting only of hand/banana.
        state['invalid'] = True
        time.sleep(.3)
        count = len(received)
        time.sleep(.4)
        assert len(received) == count  # Not a license to ignore a failed depth camera.
        state['invalid'] = False
        for key, value in [('parked', False), ('joint_stale', True), ('carry', False),
                           ('carry', None)]:
            state.update(parked=True, joint_stale=False, carry=True)
            time.sleep(.3)
            assert received[-1][1].width == 0
            state[key] = value
            time.sleep(1.0)
            assert received[-1][1].width == 64
        assert process.poll() is None
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        process.wait(timeout=5)
        output.close()


def test_carry_clear_all_obstacles_parameter(tmp_path, monkeypatch):
    """When carry_clear_all_obstacles is true, all depth points are cleared while carrying."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(190 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    executable = Path(get_package_prefix('aruco_localizer')) / \
        'lib/aruco_localizer/obstacle_depth_node'
    output = (tmp_path / 'depth_clear.log').open('w')
    process = subprocess.Popen([
        str(executable),
        '--ros-args', '-p', 'carry_clear_all_obstacles:=true',
    ], stdout=output, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('fake_depth_camera_clear')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    received = []
    node.create_subscription(PointCloud2, '/cleanup/obstacle_points',
                             lambda msg: received.append((time.monotonic(), msg)), 1)
    info_pub = node.create_publisher(CameraInfo, '/camera/camera/color/camera_info', 1)
    image_pub = node.create_publisher(Image, '/camera/camera/aligned_depth_to_color/image_raw', 1)
    info = CameraInfo(width=128, height=128, k=[1000., 0., 64., 0., 1000., 64., 0., 0., 1.])
    carry_pub = node.create_publisher(
        Bool, '/cleanup/carrying',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    joint_pub = node.create_publisher(JointState, '/joint_states', 1)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_color_optical_frame'
    transform.transform.translation.x = 0.12
    transform.transform.translation.z = 0.03
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)
    values = np.full((64, 64), 200, dtype=np.uint16)
    values[0, 0] = 1000
    values[8, 8] = 1000

    def publish():
        info_pub.publish(info)
        message = Image(height=64, width=64, encoding='16UC1', is_bigendian=1)
        message.header.frame_id = 'camera_color_optical_frame'
        now = node.get_clock().now().nanoseconds
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(now, 1000000000)
        data = values.astype('>u2')
        message.step = 64 * data.dtype.itemsize + 4
        message.data = b''.join(row.tobytes() + bytes(4) for row in data)
        image_pub.publish(message)
        carry_pub.publish(Bool(data=True))
        joint = JointState()
        joint.header.stamp = node.get_clock().now().to_msg()
        joint.name = ['joint1', 'joint2', 'joint3', 'joint4']
        joint.position = [0., -0.523, -0.523, 1.5707]
        joint.velocity = [0.] * 4
        joint_pub.publish(joint)

    node.create_timer(1 / 30, publish)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 5
        while len(received) < 3 and time.monotonic() < deadline:
            time.sleep(.02)
        assert len(received) >= 3, (tmp_path / 'depth_clear.log').read_text()
        time.sleep(0.5)
        # Even with outside obstacle pixels, carrying=True clears all depth points to 0 width.
        assert received[-1][1].width == 0
        assert process.poll() is None
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        process.wait(timeout=5)
        output.close()

