"""Tests for mask-aware robust depth sampling."""

import numpy as np
import pytest

from cleanup_perception.depth_geometry import robust_depth_sample
from cleanup_perception.depth_geometry import central_grasp_sample


def test_mask_excludes_background_depth():
    """Only object-mask pixels may influence the estimated distance."""
    depth = np.full((10, 10), 2000, dtype=np.uint16)
    depth[2:8, 2:8] = 1000
    mask = np.zeros((10, 10), dtype=bool)
    mask[2:8, 2:8] = True

    sample = robust_depth_sample(mask, depth, 0.1, 3.0, min_pixels=20)

    assert sample is not None
    assert sample.distance == pytest.approx(1.0)
    assert sample.u == pytest.approx(4.5)
    assert sample.v == pytest.approx(4.5)


def test_depth_outlier_is_rejected():
    """One invalid surface must not move a segmented object's centroid."""
    depth = np.full((8, 8), 750, dtype=np.uint16)
    depth[3, 3] = 2500
    mask = np.ones((8, 8), dtype=bool)

    sample = robust_depth_sample(mask, depth, 0.1, 3.0, min_pixels=20)

    assert sample is not None
    assert sample.distance == pytest.approx(0.75)
    assert sample.pixel_count == 63


def test_too_few_valid_mask_pixels_returns_none():
    """Sparse SAM2 masks cannot produce a grasp coordinate."""
    depth = np.full((6, 6), 1000, dtype=np.uint16)
    mask = np.zeros((6, 6), dtype=bool)
    mask[0:2, 0:2] = True

    assert robust_depth_sample(
        mask, depth, 0.1, 3.0, min_pixels=5
    ) is None


def test_body_center_does_not_follow_depth_at_tip():
    """Missing center depth is rejected instead of moving the grasp to a visible end."""
    mask = np.zeros((80, 160), dtype=bool)
    mask[25:55, 10:150] = True
    depth = np.zeros(mask.shape, dtype=np.uint16)
    depth[25:55, 125:150] = 600
    assert central_grasp_sample(mask, depth, 0.1, 3.0) is None
    depth[25:55, 55:105] = 600
    sample = central_grasp_sample(mask, depth, 0.1, 3.0)
    assert sample is not None
    assert abs(sample.u - 79.5) < 2
    assert abs(sample.v - 39.5) < 2


def test_clipped_mask_cannot_be_used_as_a_complete_centroid():
    """A bottom-clipped banana cannot yield a trustworthy full-object grasp point."""
    mask = np.zeros((80, 80), dtype=bool)
    mask[20:, 30:50] = True
    assert central_grasp_sample(mask, np.full(mask.shape, 600, np.uint16), 0.1, 3.0) is None


def test_curved_mask_selects_body_not_empty_geometric_center():
    """A crescent centroid may be outside the mask; choose a nearby thick interior."""
    rows, cols = np.indices((120, 120))
    mask = ((rows - 60) ** 2 + (cols - 60) ** 2 < 45 ** 2)
    mask &= ((rows - 60) ** 2 + (cols - 78) ** 2 > 36 ** 2)
    sample = central_grasp_sample(mask, np.full(mask.shape, 600, np.uint16), 0.1, 3.0)
    assert sample is not None
    assert mask[int(sample.v), int(sample.u)]
    assert 40 < sample.v < 80


def test_sloping_surface_depth_belongs_to_selected_ray():
    """Asymmetric missing depth cannot shift the depth value onto the near tip."""
    yy, xx = np.indices((100, 180))
    mask = (yy > 25) & (yy < 75) & (xx > 15) & (xx < 165)
    depth = (0.5 + 0.002 * xx + 0.001 * yy).astype(np.float32)
    depth[(xx > 96) & (yy > 52)] = 0
    sample = central_grasp_sample(mask, depth, 0.1, 3.0)
    assert sample is not None
    assert sample.distance == pytest.approx(0.5 + 0.002*sample.u + 0.001*sample.v, abs=1e-5)


def test_one_sided_depth_cannot_invent_body_center():
    """A central ray requires measured support on both sides, not extrapolation."""
    mask = np.zeros((100, 180), dtype=bool)
    mask[25:75, 15:165] = True
    depth = np.full(mask.shape, 600, np.uint16)
    depth[:, :92] = 0
    assert central_grasp_sample(mask, depth, 0.1, 3.0) is None
