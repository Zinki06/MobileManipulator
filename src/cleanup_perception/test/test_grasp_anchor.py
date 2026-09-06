"""Verify a clipped view never invents a center or silently changes objects."""

import copy
from types import SimpleNamespace

from geometry_msgs.msg import PointStamped
import numpy as np
import pytest
from std_msgs.msg import Header

from cleanup_perception.grasp_anchor import GraspAnchors, anchored_sample


class IdentityTF:
    """Emulate time-consistent transforms without starting hardware."""

    def transform(self, point, frame, timeout=None):
        """Change only the frame name for colocated camera and odometry."""
        result = copy.deepcopy(point)
        result.header.frame_id = frame
        return result


def test_partial_mask_needs_visible_matching_body_center():
    """Accept an observed body center, reject hidden center and mismatched depth."""
    mask = np.zeros((80, 80), dtype=bool)
    mask[25:, 25:55] = True
    depth = np.full(mask.shape, 0.5, dtype=np.float32)
    assert anchored_sample(mask, depth, 40, 40, 0.5) is not None
    assert anchored_sample(mask, depth, 40, 40, 0.6) is None
    assert anchored_sample(mask, depth, 40, 78, 0.5) is None
    mask[30:50, 30:50] = False
    assert anchored_sample(mask, depth, 40, 40, 0.5) is None


def test_anchor_age_class_and_ambiguity():
    """No YOLO id reuse can override class, age or multiple matching anchors."""
    anchors = GraspAnchors(IdentityTF())
    header = Header(frame_id='camera')
    header.stamp.sec = 10
    point = PointStamped(header=header)
    point.point.z = 0.5
    anchors.remember('banana', point, header)
    info = SimpleNamespace(k=[50., 0., 40., 0., 50., 40., 0., 0., 1.], width=80, height=80)
    mask = np.ones((80, 80), dtype=bool)
    depth = np.full(mask.shape, 0.5, dtype=np.float32)
    result = anchors.sample('banana', mask, depth, info, header)
    assert result is not None and result.u == pytest.approx(40)
    assert anchors.sample('apple', mask, depth, info, header) is None
    anchors._anchors.append(copy.deepcopy(anchors._anchors[0]))
    assert anchors.sample('banana', mask, depth, info, header) is None
    anchors._anchors.pop()
    header.stamp.sec = 31
    assert anchors.sample('banana', mask, depth, info, header) is None
    anchors.clear()
    assert not anchors._anchors


def test_close_bananas_do_not_merge_into_one_anchor():
    """Keep neighboring objects distinct even if they are less than eight cm apart."""
    anchors = GraspAnchors(IdentityTF())
    header = Header(frame_id='camera')
    header.stamp.sec = 10
    point = PointStamped(header=header)
    point.point.z = 0.5
    anchors.remember('banana', point, header, observation_key=0)
    anchors.remember('banana', point, header, observation_key=0)
    assert len(anchors._anchors) == 1
    point.point.x = 0.03
    anchors.remember('banana', point, header, observation_key=1)
    assert len(anchors._anchors) == 2
