"""Bound local height alignment using observed floor, without changing robot TF."""

import cv2
import numpy as np


def align_candidate_heights(candidates, mask, depth, k, rotation, translation, floor):
    """
    Raise underestimated heights only when a surrounding flat floor supports it.

    The fixed robot floor and all finger clearance checks remain unchanged. This
    corrects at most 15 mm upwards; it never lowers a target or adjusts X/Y.
    Missing, sloped, discontinuous or poorly supported floor leaves targets alone.
    """
    report = {'applied': False, 'reason': 'insufficient surrounding floor'}
    if not candidates:
        return report
    mask = np.asarray(mask, dtype=np.uint8)
    metres = np.asarray(depth, dtype=float)
    if np.issubdtype(depth.dtype, np.integer):
        metres = metres * 0.001
    # Exclude mask-edge mixed depths. Sample a local ring, not the whole scene.
    outer = cv2.dilate(mask, np.ones((181, 181), np.uint8)) > 0
    inner = cv2.dilate(mask, np.ones((25, 25), np.uint8)) > 0
    ring = (outer & ~inner)[::3, ::3]
    vv, uu = np.nonzero(ring)
    vv, uu = vv * 3, uu * 3
    values = metres[vv, uu]
    valid = np.isfinite(values) & (values > 0.1) & (values < 2.5)
    if len(values) < 200 or np.mean(valid) < 0.70:
        return report
    rays = np.column_stack((uu[valid], vv[valid], np.ones(valid.sum())))
    points = (rays @ np.linalg.inv(k).T * values[valid, None]) @ rotation.T + translation
    points = points[::max(1, int(np.ceil(len(points) / 4000)))]
    design = np.column_stack((points[:, :2], np.ones(len(points))))
    keep = np.abs(points[:, 2] - floor) < 0.035
    if keep.sum() < 200:
        return report
    for _ in range(6):
        coefficients, _, rank, _ = np.linalg.lstsq(design[keep], points[keep, 2], rcond=None)
        if rank != 3:
            return report
        residuals = points[:, 2] - design @ coefficients
        median = np.median(residuals[keep])
        mad = np.median(np.abs(residuals[keep] - median))
        keep = (np.abs(residuals - median) < max(0.0025, 3 * 1.4826 * mad)) & (
            np.abs(points[:, 2] - floor) < 0.035)
        if keep.sum() < 200:
            return report
    rms = float(np.sqrt(np.mean(residuals[keep] ** 2)))
    tilt = float(np.degrees(np.arctan(np.linalg.norm(coefficients[:2]))))
    report.update(plane_link1=coefficients.tolist(), residual_m=rms,
                  tilt_degrees=tilt, inliers=int(keep.sum()), inlier_fraction=float(keep.mean()))
    if keep.mean() < 0.80 or rms > 0.002 or tilt > 5.0:
        report['reason'] = 'floor quality outside alignment limits'
        return report
    xy = points[keep, :2]
    lower, upper = np.percentile(xy, [2, 98], axis=0)
    locations = np.array([candidate['observed_link1'] for candidate in candidates])
    if np.any(locations[:, :2] < lower + 0.01) or np.any(locations[:, :2] > upper - 0.01):
        report['reason'] = 'floor does not surround candidate locations'
        return report
    local_floor = np.column_stack((locations[:, :2], np.ones(len(locations)))) @ coefficients
    corrections = floor - local_floor
    report['correction_range_m'] = [float(corrections.min()), float(corrections.max())]
    if np.any(corrections < 0) or np.any(corrections > 0.015):
        report['reason'] = 'requires downward or greater than 15mm correction'
        return report
    for candidate, correction, observed_floor in zip(candidates, corrections, local_floor):
        candidate['raw_observed_link1'] = list(candidate['observed_link1'])
        candidate['observed_link1'] = list(candidate['observed_link1'])
        candidate['target_link1'] = list(candidate['target_link1'])
        candidate['observed_link1'][2] += float(correction)
        candidate['target_link1'][2] += float(correction)
        candidate['floor_correction_m'] = float(correction)
        candidate['observed_floor_link1'] = float(observed_floor)
        candidate['depth_uncertainty_m'] = max(candidate['depth_uncertainty_m'], rms + 0.001)
    report.update(applied=True, reason='surrounding floor supports bounded upward alignment')
    return report
