"""Check obstacle projection without camera or hardware I/O."""

from types import SimpleNamespace

import numpy as np

from cleanup_perception.obstacle_depth_node import depth_points


def test_depth_cloud_preserves_units_and_omits_invalid_pixels():
    """Millimetre depth becomes a metric optical cloud with no fabricated obstacles."""
    info = SimpleNamespace(width=16, height=16, k=[10, 0, 8, 0, 10, 8, 0, 0, 1])
    depth = np.full((16, 16), 1000, np.uint16)
    depth[0, 0] = 0
    points = depth_points(depth, info)
    assert points.shape == (3, 3)
    assert np.all(points[:, 2] == 1.0)
    assert np.allclose(points[-1], [0, 0, 1])
