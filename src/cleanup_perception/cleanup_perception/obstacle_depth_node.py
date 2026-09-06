"""Publish a lightweight depth cloud independently of YOLO and capture requests."""

import time

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


def depth_points(depth, info, stride=8):
    """Project valid sampled aligned-depth pixels into their optical frame."""
    height, width = depth.shape
    fx, fy = info.k[0] * width / info.width, info.k[4] * height / info.height
    cx, cy = info.k[2] * width / info.width, info.k[5] * height / info.height
    if fx <= 0 or fy <= 0:
        raise ValueError('Invalid depth intrinsics')
    rows, cols = np.mgrid[0:height:stride, 0:width:stride]
    z = depth[::stride, ::stride].astype(np.float32)
    if np.issubdtype(depth.dtype, np.integer):
        z /= 1000.0
    valid = np.isfinite(z) & (z >= 0.10) & (z <= 3.0)
    return np.column_stack(((cols[valid] - cx) * z[valid] / fx,
                            (rows[valid] - cy) * z[valid] / fy,
                            z[valid])).astype('<f4')


class ObstacleDepthNode(Node):
    """Feed near-floor obstacles to Nav2 without saving photos or running models."""

    def __init__(self):
        super().__init__('obstacle_depth')
        self._bridge = CvBridge()
        self._info = None
        self._last = 0.0
        self._publisher = self.create_publisher(PointCloud2, '/cleanup/obstacle_points', 5)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                 self._camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw',
                                 self._depth, qos_profile_sensor_data)

    def _camera_info(self, message):
        self._info = message

    def _depth(self, message):
        if self._info is None or time.monotonic() - self._last < 0.18:
            return
        self._last = time.monotonic()
        try:
            points = depth_points(self._bridge.imgmsg_to_cv2(message), self._info)
            # No valid depth is not evidence of free space: let source timeout stop motion.
            if len(points) < 20:
                return
            cloud = PointCloud2()
            cloud.header = message.header
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [PointField(name=name, offset=i * 4,
                                       datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.is_bigendian = False
            cloud.point_step, cloud.row_step = 12, 12 * len(points)
            cloud.is_dense = True
            cloud.data = points.tobytes()
            self._publisher.publish(cloud)
        except (ValueError, ZeroDivisionError) as error:
            self.get_logger().warning(f'Invalid obstacle depth: {error}')


def main(args=None):
    """Run the independent obstacle feed."""
    rclpy.init(args=args)
    node = ObstacleDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
