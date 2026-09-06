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
