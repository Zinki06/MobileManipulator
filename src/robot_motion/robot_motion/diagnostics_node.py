"""Record shared motion evidence without publishing commands or robot goals."""

from datetime import datetime
import json
import math
import os
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from robot_motion.process_metrics import ProcessMetrics


def transformed_points(points, transform):
    """Transform sensor points using a unit quaternion from TF."""
    q, t = transform.rotation, transform.translation
    v = np.array([q.x, q.y, q.z])
    uv = np.cross(v, points)
    return points + 2 * (q.w * uv + np.cross(v, uv)) + [t.x, t.y, t.z]


class MotionDiagnostics(Node):
    """Persist commands, measured motion, transforms and near-body sensor points."""

    def __init__(self):
        super().__init__('motion_diagnostics')
        root = self.declare_parameter(
            'output_directory', '/home/user/turtlebot3_ws/motion_debug').value
        directory = Path(root) / (datetime.now().strftime('run_%Y%m%d_%H%M%S_') + str(os.getpid()))
        directory.mkdir(parents=True, exist_ok=True)
        self._file = (directory / 'telemetry.jsonl').open('a', buffering=1)
        self.get_logger().info(f'Motion diagnostics: {directory}')
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._state = {}
        self._sensors = {}
        self._sensor_stamp = {}
        self._arrivals = {}
        self._metrics = ProcessMetrics()
        self.create_timer(5.0, self._performance)
        for topic in ('/cmd_vel_nav', '/cmd_vel_smoothed',
                      '/cmd_vel_collision_checked', '/cmd_vel'):
            self.create_subscription(Twist, topic,
                                     lambda msg, key=topic: self._twist(key, msg), 10)
        self.create_subscription(Odometry, '/odom', self._odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._joints,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/cleanup/obstacle_points',
                                 self._depth, qos_profile_sensor_data)
        for topic in ('/motion/events', '/aruco/correction_diagnostics', '/cleanup/events'):
            self.create_subscription(String, topic,
                                     lambda msg, key=topic: self._record(key, msg.data), 30)
        self.create_subscription(String, '/motion_guard/status',
                                 lambda msg: self._state.update(guard=msg.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(NavPath, '/plan', self._path, 1)
        self.create_timer(0.25, self._tick)

    def _record(self, kind, value):
        try:
            self._file.write(json.dumps({'time': self.get_clock().now().nanoseconds * 1e-9,
                                         'kind': kind, 'value': value}, allow_nan=False) + '\n')
        except (ValueError, OSError) as error:
            self.get_logger().error(f'Diagnostic write failed: {error}')

    def _performance(self):
        record = self._metrics.sample()
        record['topics'] = {key: {'received_hz': item[0] / record['sample_seconds'],
                                  'max_input_age_seconds': item[1]}
                            for key, item in self._arrivals.items()}
        self._arrivals.clear()
        self._record('performance', record)

    def _arrival(self, topic, header=None):
        age = (self.get_clock().now().nanoseconds * 1e-9 -
               (header.stamp.sec + header.stamp.nanosec * 1e-9)) if header else None
        count, maximum = self._arrivals.get(topic, (0, None))
        self._arrivals[topic] = (count + 1, age if maximum is None else
                                 maximum if age is None else max(age, maximum))

    def _twist(self, topic, msg):
        self._arrival(topic)
        self._state[topic] = [msg.linear.x, msg.angular.z]

    def _odom(self, msg):
        self._arrival('/odom', msg.header)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self._state['odom'] = [p.x, p.y, math.atan2(
            2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
            msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        self._state['odom_stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _joints(self, msg):
        self._arrival('/joint_states', msg.header)
        self._state['joint_positions'] = dict(zip(msg.name, msg.position))

    def _path(self, msg):
        self._record('plan', {
            'frame': msg.header.frame_id,
            'points': [[p.pose.position.x, p.pose.position.y] for p in msg.poses]})

    def _near(self, source, header, points):
        try:
            tf = self._buffer.lookup_transform('base_link', header.frame_id,
                                               rclpy.time.Time.from_msg(header.stamp))
            points = transformed_points(points, tf.transform)
            if source == 'depth':
                points = points[(points[:, 2] >= 0.08) & (points[:, 2] <= 1.5)]
            ranges = np.linalg.norm(points[:, :2], axis=1)
            near = points[ranges <= 0.22]
            self._sensors[source] = {'frame': header.frame_id, 'count_inside_22cm': len(near),
                                     'nearest_m': float(ranges.min()) if len(ranges) else None,
                                     'points_inside_22cm': near[:32].tolist()}
            self._sensor_stamp[source] = header.stamp.sec + header.stamp.nanosec * 1e-9
        except Exception as error:
            self._sensors[source] = {'error': str(error)}

    def _scan(self, msg):
        self._arrival('/scan', msg.header)
        ranges = np.asarray(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        good = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        points = np.column_stack([ranges[good]*np.cos(angles[good]),
                                  ranges[good]*np.sin(angles[good]), np.zeros(good.sum())])
        self._near('laser', msg.header, points)

    def _depth(self, msg):
        self._arrival('/cleanup/obstacle_points', msg.header)
        try:
            offsets = {f.name: f.offset for f in msg.fields}
            arrays = [np.ndarray((msg.height, msg.width),
                                 dtype='>f4' if msg.is_bigendian else '<f4',
                                 buffer=msg.data, offset=offsets[name],
                                 strides=(msg.row_step, msg.point_step)).ravel()
                      for name in ('x', 'y', 'z')]
            points = np.column_stack(arrays)
            self._near('depth', msg.header, points[np.isfinite(points).all(axis=1)])
        except (ValueError, KeyError) as error:
            self._sensors['depth'] = {'error': str(error)}

    def _tick(self):
        record = dict(self._state)
        record['sensors'] = self._sensors
        record['sensor_stamps'] = self._sensor_stamp
        for target, source in [('map', 'odom'), ('odom', 'base_link')]:
            try:
                tf = self._buffer.lookup_transform(target, source, rclpy.time.Time())
                p, q = tf.transform.translation, tf.transform.rotation
                record[target + '_to_' + source] = [
                    p.x, p.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
            except Exception as error:
                record[target + '_to_' + source] = str(error)
        try:
            self._record('telemetry', record)
        except (ValueError, OSError) as error:
            self.get_logger().error(f'Diagnostic write failed: {error}')


def main(args=None):
    """Run the passive recorder; it never performs a calibration movement."""
    rclpy.init(args=args)
    node = MotionDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
