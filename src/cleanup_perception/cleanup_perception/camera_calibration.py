"""Optional local camera extrinsics for object points; never publish global TF."""

from copy import deepcopy

from geometry_msgs.msg import PointStamped
import numpy as np


class CameraCalibration:
    """Replace the camera-to-wrist transform at the perception boundary."""

    def __init__(self, translation, rotation, camera_frame):
        self.translation = np.asarray(translation, dtype=float)
        self.rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
        self.camera_frame = camera_frame
        if (self.translation.shape != (3,) or
                not np.isfinite(self.translation).all() or
                not np.isfinite(self.rotation).all() or
                not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-5) or
                not np.isclose(np.linalg.det(self.rotation), 1.0, atol=1e-5) or
                np.linalg.norm(self.translation) > 0.20):
            raise ValueError('Invalid camera calibration')

    def to_wrist(self, point):
        """Transform an optical-frame point, preserving its acquisition timestamp."""
        if point.header.frame_id != self.camera_frame:
            raise ValueError('Calibration camera frame does not match input')
        xyz = self.rotation @ np.array([point.point.x, point.point.y, point.point.z])
        xyz += self.translation
        result = PointStamped()
        result.header = deepcopy(point.header)
        result.header.frame_id = 'link5'
        result.point.x, result.point.y, result.point.z = map(float, xyz)
        return result
