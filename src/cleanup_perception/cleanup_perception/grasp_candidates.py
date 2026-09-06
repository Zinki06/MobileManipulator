"""Generate body grasp candidates for a yaw-constrained arm without robot I/O."""

import json
import subprocess

import cv2
import numpy as np

from cleanup_perception.depth_geometry import DepthSample
from cleanup_perception.jaw_geometry import jaw_center_sample


class NoFeasibleCandidate(ValueError):
    """Keep rejected plans available for inspection without selecting a fallback."""

    def __init__(self, evaluated):
        super().__init__('No candidate has a valid hover and descent; reapproach required')
        self.evaluated = evaluated


def body_candidates(mask, depth, k, rotation, translation, forward_offset=0.0):
    """
    Sample interior body sections and estimate transverse width and local direction.

    All dimensions are metres in link1. Visible surface width is not a complete
    contact or collision model. Forward calibration is an explicit caller input.
    """
    mask, depth = np.asarray(mask), np.asarray(depth)
    k = np.asarray(k, dtype=float).reshape(3, 3)
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    translation = np.asarray(translation, dtype=float).reshape(3)
    if mask.ndim != 2 or mask.shape != depth.shape:
        raise ValueError('Aligned mask and depth required')
    if not all(np.isfinite(a).all() for a in (k, rotation, translation)):
        raise ValueError('Non-finite geometry')
    if not np.isfinite(forward_offset) or abs(forward_offset) > 0.03:
        raise ValueError('Forward offset exceeds experimental calibration bound')
    metres = depth.astype(float)
    if np.issubdtype(depth.dtype, np.integer):
        metres *= 0.001
    interior = cv2.erode(mask.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    yy, xx = np.nonzero(interior & np.isfinite(metres) &
                        (metres > 0.1) & (metres < 2.5))
    if len(xx) < 200:
        return []
    rays = np.column_stack((xx, yy, np.ones(len(xx)))) @ np.linalg.inv(k).T
    points = (rays * metres[yy, xx, None]) @ rotation.T + translation
    center = np.median(points[:, :2], axis=0)
    _, axes = np.linalg.eigh(np.cov(points[:, :2], rowvar=False))
    axis = axes[:, -1]
    if axis[0] < 0:
        axis = -axis
    longitudinal = (points[:, :2] - center) @ axis
    ends = np.percentile(longitudinal, [2, 98])
    candidates, pixels = [], set()
    for percentile in range(30, 71, 5):
        section = np.percentile(longitudinal, percentile)
        band = np.abs(longitudinal - section) < 0.003
        if np.count_nonzero(band) < 60:
            continue
        midpoint = np.median(points[band], axis=0)
        index = np.argmin(np.linalg.norm(points - midpoint, axis=1))
        seed = DepthSample(float(xx[index]), float(yy[index]),
                           float(metres[yy[index], xx[index]]), 0., int(band.sum()))
        try:
            sample, detail = jaw_center_sample(
                mask, depth, k, rotation, translation, seed, forward_offset)
        except ValueError:
            continue
        pixel = (sample.u, sample.v)
        if pixel in pixels:
            continue
        pixels.add(pixel)
        observed = np.asarray(detail['jaw_center_link1'])
        actual_section = (observed[:2] - center) @ axis
        end_margin = min(actual_section - ends[0], ends[1] - actual_section)
        width = detail['transverse_width_m']
        # Reserve 10mm from nominal 75mm opening; exclude thin ends and broad spans.
        if not 0.025 <= width <= 0.065 or end_margin < 0.020:
            continue
        target = observed.copy()
        target[0] += forward_offset
        yaw = np.arctan2(target[1], target[0] - 0.012)
        near = points[np.linalg.norm(points[:, :2] - observed[:2], axis=1) < 0.025]
        if len(near) < 60:
            continue
        values, vectors = np.linalg.eigh(np.cov(near[:, :2], rowvar=False))
        tangent = vectors[:, -1]
        mismatch = float(np.degrees(np.arccos(np.clip(
            abs(tangent @ np.array([np.cos(yaw), np.sin(yaw)])), 0., 1.))))
        candidates.append({
            'pixel': list(pixel), 'depth_m': sample.distance,
            'observed_link1': observed.tolist(), 'target_link1': target.tolist(),
            'forward_offset_m': forward_offset, 'body_percentile': percentile,
            'width_m': width, 'end_margin_m': float(end_margin),
            'local_axis_mismatch_degrees': mismatch,
            'local_axis_confidence_ratio': float(values[1] / max(values[0], 1e-12)),
            'depth_uncertainty_m': sample.uncertainty,
        })
    return candidates


def choose_candidate(candidates, floor, start, planner):
    """Rank candidates only after the read-only planner validates hover and descent."""
    if not candidates:
        raise ValueError('No body section fits width, end-margin and depth constraints')
    request = ''.join(' '.join(map(str, c['target_link1'] + [floor] + list(start))) + '\n'
                      for c in candidates)
    completed = subprocess.run([str(planner)], input=request, text=True,
                               capture_output=True, check=True, timeout=5)
    plans = [json.loads(line) for line in completed.stdout.splitlines()]
    if len(plans) != len(candidates):
        raise ValueError('Planner response count mismatch')
    evaluated = []
    for candidate, plan in zip(candidates, plans):
        item = {key: value for key, value in candidate.items() if key not in ('plan', 'score')}
        item['plan'] = plan
        if plan.get('feasible'):
            # Balance body support, centering, aperture margin and local orientation.
            item['score'] = (
                1000. * plan['body_depth']
                - 0.12 * abs(candidate['body_percentile'] - 50)
                - 0.08 * candidate['local_axis_mismatch_degrees']
                - 0.15 * abs(plan['pitch_degrees'] + 45.)
                - 100. * candidate['width_m']
                - 1000. * candidate['depth_uncertainty_m'])
        evaluated.append(item)
    feasible = [c for c in evaluated if c['plan'].get('feasible')]
    if not feasible:
        raise NoFeasibleCandidate(evaluated)
    return max(feasible, key=lambda c: c['score']), evaluated
