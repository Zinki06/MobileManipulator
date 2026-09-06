"""Experimental jaw-direction centering for stationary, open-gripper hover trials."""

import cv2
import numpy as np

from cleanup_perception.depth_geometry import central_grasp_sample


def jaw_center_sample(mask, depth, k, rotation, translation, original,
                      forward_offset=0.030):
    """
    Recenter a valid body sample between opposing transverse surface boundaries.

    Rotation/translation map optical points into link1. This estimates the visible
    surface footprint, not full finger collision clearance or grasp feasibility.
    """
    mask = np.asarray(mask, dtype=np.uint8)
    k = np.asarray(k, dtype=float).reshape(3, 3)
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    translation = np.asarray(translation, dtype=float).reshape(3)
    if original is None or mask.shape != depth.shape:
        raise ValueError('A valid original body sample and aligned mask are required')
    if not all(np.isfinite(a).all() for a in (k, rotation, translation)):
        raise ValueError('Non-finite geometry')
    inverse_k = np.linalg.inv(k)
    metres = depth.astype(float)
    if np.issubdtype(depth.dtype, np.integer):
        metres /= 1000.0
    interior = cv2.erode(mask, np.ones((5, 5), dtype=np.uint8)) > 0
    yy, xx = np.nonzero(interior & np.isfinite(metres) &
                        (metres > 0.1) & (metres < 2.5))
    camera = (np.column_stack((xx, yy, np.ones(len(xx)))) @ inverse_k.T)
    points = (camera * metres[yy, xx, None]) @ rotation.T + translation

    def local(sample):
        ray = inverse_k @ np.array([sample.u, sample.v, 1.0])
        return rotation @ (ray * sample.distance) + translation

    target = local(original)
    yaw = np.arctan2(target[1], target[0] + forward_offset - 0.012)
    approach = np.array([np.cos(yaw), np.sin(yaw)])
    closing = np.array([-np.sin(yaw), np.cos(yaw)])
    relative = points[:, :2] - target[:2]
    band = ((np.abs(relative @ approach) < 0.005) &
            (np.abs(points[:, 2] - target[2]) < 0.030))
    if np.count_nonzero(band) < 60:
        raise ValueError('Insufficient transverse body support')
    sides = np.percentile(relative[band] @ closing, [2, 98])
    shift = float(np.mean(sides))
    width = float(sides[1] - sides[0])
    if not 0.005 < width < 0.12 or abs(shift) > 0.020:
        raise ValueError('Unbounded jaw-center adjustment')
    candidate = target.copy()
    candidate[:2] += shift * closing
    optical = rotation.T @ (candidate - translation)
    if optical[2] <= 0:
        raise ValueError('Candidate behind camera')
    projected = k @ optical
    u, v = projected[:2] / projected[2]
    h, w = depth.shape
    if not 3 <= u < w - 3 or not 3 <= v < h - 3 or not interior[int(v), int(u)]:
        raise ValueError('Jaw center outside observed body')
    yy, xx = np.indices(depth.shape)
    region = (mask > 0) & ((xx - u) ** 2 + (yy - v) ** 2 <= 8 ** 2)
    sample = central_grasp_sample(region, depth, 0.1, 2.5, 30)
    if sample is None:
        raise ValueError('No reliable depth at jaw center')
    final = local(sample)
    if np.linalg.norm(final - target) > 0.025:
        raise ValueError('Depth changed the jaw center excessively')
    return sample, {
        'original_link1': target.tolist(),
        'jaw_center_link1': final.tolist(),
        'transverse_sides_m': sides.tolist(),
        'transverse_width_m': width,
        'transverse_shift_m': shift,
        'jaw_center_delta_link1': (final - target).tolist(),
    }
