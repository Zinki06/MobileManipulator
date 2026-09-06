"""Re-observe a previously verified body center without inventing a clipped centroid."""

from dataclasses import dataclass
import math

import cv2
from geometry_msgs.msg import PointStamped
import numpy as np
from rclpy.duration import Duration

from cleanup_perception.depth_geometry import DepthSample, robust_depth_sample


def anchored_sample(mask, depth, u, v, predicted_depth):
    """Require the predicted body center inside the current mask with matching depth."""
    height, width = depth.shape
    if not all(math.isfinite(x) for x in (u, v, predicted_depth)):
        return None
    if not (8 <= u < width - 8 and 8 <= v < height - 8):
        return None
    interior = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    if interior[int(v), int(u)] < 4.0:
        return None
    yy, xx = np.indices(depth.shape)
    region = mask & ((xx-u)**2 + (yy-v)**2 <= 25)
    sample = robust_depth_sample(region, depth, 0.10, 2.5, 20)
    if sample is None or abs(sample.distance-predicted_depth) > 0.025 or sample.uncertainty > 0.01:
        return None
    return DepthSample(u, v, sample.distance, max(sample.uncertainty, 0.01), sample.pixel_count)


@dataclass
class Anchor:
    """A full-mask body center and camera origin stored in continuous odometry."""

    class_name: str
    point: PointStamped
    origin: PointStamped
    stamp: float
    observation_key: object = None


class GraspAnchors:
    """Keep short-lived full-body observations; partial observations never refresh age."""

    def __init__(self, tf_buffer):
        self._tf = tf_buffer
        self._anchors = []

    def clear(self):
        """Invalidate observations across missions or any attempted object manipulation."""
        self._anchors.clear()

    def _transform(self, point, frame):
        return self._tf.transform(point, frame, timeout=Duration(seconds=0.10))

    def remember(self, class_name, point, camera_header, observation_key=None):
        """Anchor only a freshly validated full-mask center, never a bbox center."""
        try:
            odom = self._transform(point, 'odom')
            origin = PointStamped(header=camera_header)
            origin = self._transform(origin, 'odom')
            stamp = camera_header.stamp.sec + camera_header.stamp.nanosec*1e-9
            # Only merge repeated frames of the same confirmed object in this
            # capture. Nearby same-class objects must remain separate candidates.
            self._anchors = [a for a in self._anchors if 0 <= stamp-a.stamp < 20.0 and not (
                observation_key is not None and a.observation_key == observation_key)]
            self._anchors.append(Anchor(class_name, odom, origin, stamp, observation_key))
            self._anchors = self._anchors[-32:]
        except Exception:
            # Lack of odometry TF must disable this fallback, not weaken grasp checks.
            return

    def sample(self, class_name, mask, depth, info, header):
        """Return a center only when exactly one recent anchor matches current evidence."""
        stamp = header.stamp.sec + header.stamp.nanosec*1e-9
        candidates = []
        for anchor in self._anchors:
            if anchor.class_name != class_name or not 0 <= stamp-anchor.stamp < 20.0:
                continue
            try:
                current_origin = self._transform(PointStamped(header=header), 'odom')
                if math.dist((current_origin.point.x, current_origin.point.y,
                              current_origin.point.z),
                             (anchor.origin.point.x, anchor.origin.point.y,
                              anchor.origin.point.z)) > 0.15:
                    continue
                stamped = PointStamped()
                stamped.header.frame_id, stamped.header.stamp = 'odom', header.stamp
                stamped.point = anchor.point.point
                projected = self._transform(stamped, header.frame_id).point
                if projected.z <= 0:
                    continue
                h, w = depth.shape
                u = (info.k[0]*projected.x/projected.z + info.k[2])*w/info.width
                v = (info.k[4]*projected.y/projected.z + info.k[5])*h/info.height
                sample = anchored_sample(mask, depth, u, v, projected.z)
                if sample is not None:
                    candidates.append(sample)
            except Exception:
                continue
        return candidates[0] if len(candidates) == 1 else None
