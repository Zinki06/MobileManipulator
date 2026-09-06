"""Verify carried-object localization using synthetic images and wheel odometry."""

import os
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
import cv2
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32, String
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def test_carry_freezes_correction_but_keeps_odometry(tmp_path, monkeypatch):
    """Continue TF with no RGB while holding; resume marker corrections after release."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(115 + os.getpid() % 10))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    markers = tmp_path / 'markers.yaml'
    markers.write_text('markers:\n  - {id: 1, size: 0.2, x: 0.6, y: 0.0, yaw: 0.0}\n')
    rclpy.init()
    node = rclpy.create_node('fake_carry_localization_sensors')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    carry = node.create_publisher(Bool, '/cleanup/carrying', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    camera = node.create_publisher(Image, '/camera/camera/color/image_raw', 1)
    info_pub = node.create_publisher(CameraInfo, '/camera/camera/color/camera_info', 1)
    odom_pub = node.create_publisher(Odometry, '/odom', 1)
    expected = node.create_publisher(Int32, '/aruco/expected_marker_id', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    expected.publish(Int32(data=1))
    modes, corrections = [], []
    node.create_subscription(String, '/aruco/localization_mode',
                             lambda msg: modes.append(msg.data), 10)

    def transforms(msg):
        corrections.extend(t for t in msg.transforms if t.header.frame_id == 'map' and
                           t.child_frame_id == 'odom')

    node.create_subscription(TFMessage, '/tf', transforms, 10)
    dynamic = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'camera_optical'
    transform.transform.translation.x, transform.transform.translation.z = .6, 1.0
    transform.transform.rotation.x = 1.0
    static.sendTransform(transform)
    data = np.full((480, 640, 3), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    draw_marker = getattr(cv2.aruco, 'generateImageMarker', None)
    if draw_marker is None:
        draw_marker = cv2.aruco.drawMarker
    marker = draw_marker(dictionary, 1, 120)
    data[180:300, 260:380] = marker[:, :, None]
    info = CameraInfo(width=640, height=480, k=[600., 0., 320., 0., 600., 240., 0., 0., 1.],
                      d=[0.] * 5)
    state = {'carry': False, 'image': True, 'x': 0.0}

    def publish():
        now = node.get_clock().now()
        pose = TransformStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id, pose.child_frame_id = 'odom', 'base_link'
        pose.transform.rotation.w = 1.0
        pose.transform.translation.x = state['x']
        dynamic.sendTransform(pose)
        odom = Odometry()
        odom.header = pose.header
        odom.pose.pose.position.x = state['x']
        odom_pub.publish(odom)
        carry.publish(Bool(data=state['carry']))
        info_pub.publish(info)
        if state['image']:
            image = Image(height=480, width=640, encoding='bgr8', step=1920)
            image.header.frame_id = 'camera_optical'
            ns = now.nanoseconds - 50000000
            image.header.stamp.sec, image.header.stamp.nanosec = divmod(ns, 1000000000)
            image.data = data.tobytes()
            camera.publish(image)

    node.create_timer(.04, publish)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    executable = Path(get_package_prefix('aruco_localizer')) / \
        'lib/aruco_localizer/aruco_localizer_node'
    output = (tmp_path / 'localizer.log').open('w')
    process = subprocess.Popen([str(executable), '--ros-args', '-p',
                                f'marker_yaml_path:={markers}'], stdout=output,
                               stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 8
        while 'MARKER_CORRECTED' not in modes and time.monotonic() < deadline:
            time.sleep(.02)
        assert 'MARKER_CORRECTED' in modes, (tmp_path / 'localizer.log').read_text()
        state['carry'] = True
        deadline = time.monotonic() + 3
        while modes[-1] != 'DEAD_RECKONING' and time.monotonic() < deadline:
            time.sleep(.02)
        assert modes[-1] == 'DEAD_RECKONING', (tmp_path / 'localizer.log').read_text()
        before = corrections[-1].transform
        state['image'], state['x'] = False, .12
        count = len(corrections)
        time.sleep(.7)
        assert len(corrections) > count + 10
        assert corrections[-1].transform == before
        assert modes[-1] == 'DEAD_RECKONING'
        assert corrections[-1].header.stamp != corrections[count - 1].header.stamp
        # Even returning RGB cannot shift the map during transport.
        state['image'] = True
        time.sleep(.5)
        assert corrections[-1].transform == before
        state['carry'] = False
        time.sleep(.8)
        assert modes[-1] == 'MARKER_CORRECTED'
        assert abs(corrections[-1].transform.translation.x - before.translation.x) > .005
        assert 'DEGRADED' not in modes
    finally:
        process.terminate()
        process.wait(timeout=5)
        output.close()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
