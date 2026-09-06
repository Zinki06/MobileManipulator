"""Exercise real collision-monitor and guard nodes on an isolated, hardware-free domain."""

import os
import math
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('balanced', [False, True])
def test_collision_timeout_speed_and_map_jump(tmp_path, monkeypatch, balanced):
    """Real sensor data gates motion, speed is bounded, and a map jump latches stop."""
    monkeypatch.setenv('ROS_DOMAIN_ID', str(215 + os.getpid() % 15))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    package = Path(__file__).parents[1]
    guard = Path(get_package_prefix('aruco_localizer')) / 'lib/aruco_localizer/motion_guard_node'
    monitor = Path(get_package_prefix('nav2_collision_monitor')) / \
        'lib/nav2_collision_monitor/collision_monitor'
    outputs = [(tmp_path / f'{name}.log').open('w') for name in ('guard', 'monitor')]
    guard_args = [str(guard)]
    if balanced:
        guard_args += ['--ros-args', '--params-file', str(package / 'config/performance.yaml')]
    processes = [subprocess.Popen(guard_args, stdout=outputs[0], stderr=subprocess.STDOUT),
                 subprocess.Popen([str(monitor), '--ros-args', '--params-file',
                                   str(package / 'config/motion_safety.yaml')],
                                  stdout=outputs[1], stderr=subprocess.STDOUT)]
    rclpy.init()
    node = rclpy.create_node('fake_safety_sensors')
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    tf = TransformBroadcaster(node)
    commands = node.create_publisher(Twist, '/cmd_vel_smoothed', 10)
    laser = node.create_publisher(LaserScan, '/scan', 10)
    cloud = node.create_publisher(PointCloud2, '/cleanup/obstacle_points', 10)
    mode = node.create_publisher(String, '/aruco/localization_mode', 10)
    carrying = node.create_publisher(Bool, '/cleanup/carrying',
                                     QoSProfile(depth=1,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
    received = []
    statuses = []
    node.create_subscription(String, '/motion_guard/status',
                             lambda msg: statuses.append(msg.data), 50)
    state = {'obstacle': False, 'depth': True, 'command': True, 'jump': False,
             'laser': True, 'depth_frame': 'base_link', 'pivot': False,
             'carrying': False, 'empty_cloud': False, 'laser_obstacle': False}
    node.create_subscription(Twist, '/cmd_vel',
                             lambda msg: received.append((time.monotonic(), msg.linear.x)), 50)

    def publish():
        stamp = node.get_clock().now().to_msg()
        for parent, child in [('map', 'odom'), ('odom', 'base_link')]:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.rotation.w = 1.0
            if parent == 'odom':
                transform.transform.translation.x = 10.0
            if state['pivot'] and parent == 'map':
                transform.transform.translation.x = 10 - 10 * math.cos(0.16)
                transform.transform.translation.y = -10 * math.sin(0.16)
                transform.transform.rotation.z = math.sin(0.08)
                transform.transform.rotation.w = math.cos(0.08)
            if state['jump'] and parent == 'map':
                transform.transform.translation.x = 1.5
            tf.sendTransform(transform)
        mode.publish(String(data='DEAD_RECKONING' if state['carrying'] else 'MARKER_CORRECTED'))
        carrying.publish(Bool(data=state['carrying']))
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'base_link'
        scan.angle_min, scan.angle_increment = -3.14, 0.01745
        scan.range_min, scan.range_max = 0.05, 5.0
        scan.ranges = [3.0] * 360
        if state['laser_obstacle']:
            scan.ranges = [3.0] * 175 + [0.18] * 10 + [3.0] * 175
        if state['laser']:
            laser.publish(scan)
        if state['depth']:
            points = np.tile([0.18 if state['obstacle'] else 2.0, 0.0, 0.15], (32, 1))
            msg = PointCloud2()
            msg.header = scan.header
            msg.header.frame_id = state['depth_frame']
            msg.height, msg.width = 1, 32
            msg.fields = [PointField(name=name, offset=i * 4,
                                     datatype=PointField.FLOAT32, count=1)
                          for i, name in enumerate(('x', 'y', 'z'))]
            msg.point_step, msg.row_step = 12, 384
            msg.data = points.astype('<f4').tobytes()
            if state['empty_cloud']:
                msg.width, msg.row_step, msg.data = 0, 0, b''
            cloud.publish(msg)
        if state['command']:
            cmd = Twist()
            cmd.linear.x = 0.22
            commands.publish(cmd)

    node.create_timer(0.02, publish)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def output_is_stopped():
        recent = [v for t, v in received if t > time.monotonic() - 0.15]
        return len(recent) >= 3 and all(v == 0.0 for v in recent)

    try:
        client = node.create_client(ChangeState, '/collision_monitor/change_state')
        assert client.wait_for_service(timeout_sec=10)
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            req = ChangeState.Request()
            req.transition.id = transition
            future = client.call_async(req)
            deadline = time.monotonic() + 10
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            assert future.done() and future.result().success
        time.sleep(1.5)
        assert any(v > 0.05 for _, v in received), (tmp_path / 'guard.log').read_text()
        cap = .18 if balanced else .10
        assert max(v for _, v in received) > cap * .9
        assert all(0.0 <= v <= cap + .00001 for _, v in received)
        state['pivot'] = True
        time.sleep(0.5)
        assert not any(s.startswith('FAULT:') for s in statuses)
        assert any(v > 0.0 for t, v in received if t > time.monotonic() - 0.2)
        state['pivot'] = False
        time.sleep(0.5)
        state['obstacle'] = True
        time.sleep(0.5)
        assert output_is_stopped(), 'depth obstacle failed to stop translation'
        assert 'BLOCKED: collision monitor stopped motion' in statuses
        state['obstacle'], state['depth'] = False, False
        time.sleep(0.8)
        assert output_is_stopped(), 'stale depth did not inhibit commands'
        assert 'BLOCKED: depth stale' in statuses
        assert '[MOTION_SENSOR]' in (tmp_path / 'guard.log').read_text()
        state['depth'] = True
        time.sleep(1.0)
        assert any(v > 0.0 for t, v in received if t > time.monotonic() - 0.3)
        state['laser'] = False
        time.sleep(0.8)
        assert output_is_stopped(), 'stale laser did not inhibit commands'
        assert 'BLOCKED: laser stale' in statuses
        state['laser'], state['depth_frame'] = True, 'missing_camera_frame'
        time.sleep(0.5)
        assert output_is_stopped(), 'untransformable depth did not inhibit commands'
        assert 'BLOCKED: depth TF unavailable' in statuses
        state['depth_frame'] = 'base_link'
        time.sleep(0.8)
        state['carrying'], state['empty_cloud'] = True, True
        time.sleep(.8)
        assert any(v > 0.0 for t, v in received if t > time.monotonic() - .2)
        state['laser_obstacle'] = True
        time.sleep(.5)
        assert output_is_stopped(), 'carrying must retain LiDAR collision protection'
        state['laser_obstacle'] = False
        state['empty_cloud'], state['obstacle'] = False, True
        time.sleep(.5)
        assert output_is_stopped(), 'carrying must retain external depth obstacles'
        state['empty_cloud'], state['obstacle'], state['carrying'] = True, False, False
        time.sleep(.8)
        assert output_is_stopped(), 'empty depth requires an active verified carrying lease'
        state['empty_cloud'] = False
        time.sleep(.8)
        state['command'] = False
        time.sleep(0.5)
        assert output_is_stopped(), 'stale commands did not stop'
        state['command'], state['jump'] = True, True
        time.sleep(0.5)
        assert output_is_stopped(), 'map reset did not stop'
        state['jump'] = False
        time.sleep(0.5)
        assert output_is_stopped(), 'fault incorrectly auto-resumed'
    finally:
        executor.shutdown()
        thread.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
        for process, output in zip(processes, outputs):
            process.terminate()
            process.wait(timeout=10)
            output.close()
