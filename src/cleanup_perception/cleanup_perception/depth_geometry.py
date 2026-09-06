"""Pure depth sampling utilities shared by bbox and SAM2 observations."""

from dataclasses import dataclass

import numpy as np
import cv2


@dataclass(frozen=True)
class DepthSample:
    """Robust image-space location and depth for one segmented object."""

    u: float
    v: float
    distance: float
    uncertainty: float
    pixel_count: int


def robust_depth_sample(mask, depth, min_depth, max_depth, min_pixels=20):
    """Estimate an object point from masked depth with MAD outlier removal."""
    mask_array = np.asarray(mask, dtype=bool)
    depth_array = np.asarray(depth)
    if mask_array.ndim != 2 or depth_array.ndim != 2:
        raise ValueError('mask and depth must be two-dimensional')
    if mask_array.shape != depth_array.shape:
        raise ValueError('mask and depth shapes must match')
    if min_pixels < 1 or min_depth <= 0.0 or max_depth <= min_depth:
        raise ValueError('depth sampling limits are invalid')

    if np.issubdtype(depth_array.dtype, np.integer):
        depth_metres = depth_array.astype(np.float32) / 1000.0
    else:
        depth_metres = depth_array.astype(np.float32)

    valid_mask = (
        mask_array & np.isfinite(depth_metres) &
        (depth_metres >= min_depth) & (depth_metres <= max_depth)
    )
    rows, columns = np.nonzero(valid_mask)
    if rows.size < min_pixels:
        return None

    values = depth_metres[rows, columns]
    center = float(np.median(values))
    absolute_deviation = np.abs(values - center)
    mad = float(np.median(absolute_deviation))
    tolerance = max(0.03, 3.0 * mad)
    inliers = absolute_deviation <= tolerance
    if np.count_nonzero(inliers) < min_pixels:
        return None

    inlier_rows = rows[inliers]
    inlier_columns = columns[inliers]
    inlier_depth = values[inliers]
    distance = float(np.median(inlier_depth))
    uncertainty = float(np.median(np.abs(inlier_depth - distance)))
    return DepthSample(
        u=float(np.median(inlier_columns)),
        v=float(np.median(inlier_rows)),
        distance=distance,
        uncertainty=uncertainty,
        pixel_count=int(inlier_depth.size),
    )


def central_grasp_sample(mask, depth, min_depth, max_depth, min_pixels=20):
    """Choose the body's interior nearest the full-mask centroid, never a depth-valid tip."""
    mask = np.asarray(mask, dtype=np.uint8)
    if mask.shape != depth.shape or not np.any(mask):
        return None
    # A clipped mask has no observable full-object centroid.
    if (np.any(mask[:3]) or np.any(mask[-3:]) or
            np.any(mask[:, :3]) or np.any(mask[:, -3:])):
        return None
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return None
    component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    body = labels == component
    rows, cols = np.nonzero(body)
    if len(rows) < min_pixels:
        return None
    center_u, center_v = float(np.mean(cols)), float(np.mean(rows))
    thickness = cv2.distanceTransform(body.astype(np.uint8), cv2.DIST_L2, 5)
    # For elongated bodies, exclude both ends along the principal length axis.
    # A thick rounded end must not win just because it has more valid depth.
    coordinates = np.column_stack((cols - center_u, rows - center_v))
    eigenvalues, axes = np.linalg.eigh(np.cov(coordinates, rowvar=False))
    projection = coordinates @ axes[:, -1]
    central = body.copy()
    if eigenvalues[-1] > 2.0 * max(eigenvalues[0], 1e-6):
        lower, upper = np.quantile(projection, [0.30, 0.70])
        central[rows, cols] = (projection >= lower) & (projection <= upper)
    core_y, core_x = np.nonzero(
        central & (thickness >= max(1.0, float(thickness.max()) * 0.5)))
    if not len(core_x):
        return None
    nearest = np.argmin((core_x - center_u) ** 2 + (core_y - center_v) ** 2)
    u, v = float(core_x[nearest]), float(core_y[nearest])
    # Use depth only at the chosen central body, not wherever depth happens to exist.
    radius = max(3.0, float(thickness[int(v), int(u)]) * 0.7)
    yy, xx = np.indices(body.shape)
    region = body & ((xx - u) ** 2 + (yy - v) ** 2 <= radius ** 2)
    sample = robust_depth_sample(region, depth, min_depth, max_depth, min_pixels)
    if sample is None:
        return None
    # A median over a sloping surface belongs to the median valid pixel, not
    # necessarily (u,v). Fit local depth and evaluate at the actual selected ray.
    metres = np.asarray(depth, dtype=np.float64)
    if np.issubdtype(depth.dtype, np.integer):
        metres = metres / 1000.0
    valid = region & np.isfinite(metres) & (metres >= min_depth) & (metres <= max_depth)
    ys, xs = np.nonzero(valid)
    if len(xs) < min_pixels or np.min((xs-u)**2 + (ys-v)**2) > 9:
        return None
    design = np.column_stack((xs-u, ys-v, np.ones(len(xs))))
    values = metres[ys, xs]
    keep = np.ones(len(xs), dtype=bool)
    for _ in range(3):
        if np.count_nonzero(keep) < min_pixels:
            return None
        fit, _, rank, _ = np.linalg.lstsq(design[keep], values[keep], rcond=None)
        if rank != 3:
            return None
        residuals = np.abs(values - design @ fit)
        keep = residuals <= max(0.003, 3.0 * float(np.median(residuals)))
    # Reject one-sided support: a fit cannot invent central depth from an end.
    dx, dy = xs[keep]-u, ys[keep]-v
    if not (np.any(dx < -1) and np.any(dx > 1) and
            np.any(dy < -1) and np.any(dy > 1)):
        return None
    uncertainty = float(np.sqrt(np.mean(residuals[keep] ** 2)))
    if uncertainty > 0.008 or not min_depth <= fit[2] <= max_depth:
        return None
    return DepthSample(u, v, float(fit[2]), uncertainty, int(np.count_nonzero(keep)))
