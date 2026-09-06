"""Check geometric recentering and missing-depth rejection without robot I/O."""

import numpy as np
import pytest

from cleanup_perception.depth_geometry import DepthSample
from cleanup_perception.jaw_geometry import jaw_center_sample


def fixture():
    """Return a flat rectangle with a deliberately off-center body sample."""
    mask = np.zeros((100, 140), np.uint8)
    mask[25:75, 30:110] = 1
    depth = np.full(mask.shape, 300, np.uint16)
    k = np.array([[1000., 0., 70.], [0., 1000., 50.], [0., 0., 1.]])
    rotation = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
    translation = np.array([0.3, 0., -0.37])
    original = DepthSample(90., 50., 0.3, 0., 100)
    return mask, depth, k, rotation, translation, original


def test_centers_between_opposing_sides():
    """A sample offset along closing direction returns to the rectangle midpoint."""
    sample, info = jaw_center_sample(*fixture())
    assert abs(sample.u - 69.5) < 2
    assert abs(sample.v - 50) < 2
    assert abs(sample.distance - 0.3) < 1e-6
    assert abs(info['jaw_center_delta_link1'][1] - 0.006) < 0.001


def test_does_not_invent_depth_at_new_center():
    """A central depth hole cannot be replaced by depth from an object end."""
    args = list(fixture())
    args[1][40:61, 58:82] = 0
    with pytest.raises(ValueError):
        jaw_center_sample(*args)


def test_rejects_unbounded_recentering():
    """Do not redirect a trial to another distant part of the mask."""
    args = list(fixture())
    args[2][0, 0] = 200.
    with pytest.raises(ValueError):
        jaw_center_sample(*args)


def test_cropped_center_matches_full_image_sampling():
    """Cropping and cached projection must preserve the selected physical ray."""
    from cleanup_perception.depth_geometry import DepthSample, central_grasp_sample
    from cleanup_perception.jaw_geometry import prepare_jaw_geometry

    mask = np.zeros((180, 320), dtype=np.uint8)
    mask[50:120, 30:290] = 1
    depth = np.full(mask.shape, 300, dtype=np.uint16)
    k = np.array([[500., 0., 160.], [0., 500., 90.], [0., 0., 1.]])
    rotation = np.diag([1., -1., -1.])
    translation = np.array([.3, 0., .23])
    prepared = prepare_jaw_geometry(mask, depth, k, rotation, translation)
    seed = DepthSample(155., 78., .3, 0., 100)
    optimized, detail = jaw_center_sample(
        mask, depth, k, rotation, translation, seed, .03, prepared=prepared)
    original = np.array(detail['original_link1'])
    yaw = np.arctan2(original[1], original[0] + .03 - .012)
    candidate = original.copy()
    candidate[:2] += detail['transverse_shift_m'] * np.array([-np.sin(yaw), np.cos(yaw)])
    projected = k @ (rotation.T @ (candidate - translation))
    u, v = projected[:2] / projected[2]
    yy, xx = np.indices(mask.shape)
    region = (mask > 0) & ((xx-u)**2 + (yy-v)**2 <= 64)
    reference = central_grasp_sample(region, depth, .1, 2.5, 30)
    assert (optimized.u, optimized.v) == (reference.u, reference.v)
    assert abs(optimized.distance - reference.distance) < 1e-12
    assert abs(optimized.uncertainty - reference.uncertainty) < 1e-12
    assert optimized.pixel_count == reference.pixel_count
