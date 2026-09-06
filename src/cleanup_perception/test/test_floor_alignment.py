"""Verify floor-supported height correction without robot or camera access."""

from copy import deepcopy
import json
from types import SimpleNamespace

import numpy as np
import pytest

from cleanup_perception.floor_alignment import align_candidate_heights


def scene(bias=0.011, slope=0.):
    """Render a known plane below an object using a downward optical camera."""
    mask = np.zeros((240, 320), np.uint8)
    mask[90:150, 100:220] = 1
    k = np.array([[500., 0., 160.], [0., 500., 120.], [0., 0., 1.]])
    v, u = np.indices(mask.shape)
    depth = (0.331 + bias) / (1 + slope * (u - 160) / 500.)
    depth[mask > 0] = 0.310
    rotation = np.diag([1., -1., -1.])
    translation = np.array([0.3, 0., 0.23])
    candidates = [{'observed_link1': [0.3, 0., -0.080],
                   'target_link1': [0.33, 0., -0.080], 'depth_uncertainty_m': 0.0005}]
    return candidates, mask, depth, k, rotation, translation, -0.101


@pytest.mark.parametrize('millimetres', [9.17, 11.04])
def test_recovers_floor_relative_height_without_changing_xy(millimetres):
    """The two observed station biases must not masquerade as a thin banana."""
    args = scene(millimetres / 1000)
    report = align_candidate_heights(*args)
    assert report['applied']
    candidate = args[0][0]
    assert candidate['raw_observed_link1'] == [0.3, 0., -0.080]
    assert candidate['observed_link1'] == pytest.approx([0.3, 0., -0.080 + millimetres / 1000])
    assert np.subtract(candidate['target_link1'], candidate['observed_link1']) == pytest.approx(
        [0.03, 0., 0.])
    assert candidate['depth_uncertainty_m'] >= 0.001


@pytest.mark.parametrize('case', ['missing', 'downward', 'large', 'tilted', 'step', 'one_side'])
def test_unsupported_floor_cannot_change_a_target(case):
    """Reject unsafe height extrapolation and preserve the original fallback target."""
    args = list(scene(-0.004 if case == 'downward' else
                      0.025 if case == 'large' else 0.011,
                      0.15 if case == 'tilted' else 0.))
    if case == 'missing':
        args[2][:] = 0.
    elif case == 'step':
        args[2][:120] += 0.025
    elif case == 'one_side':
        args[2][:, :170] = 0.
    before = deepcopy(args[0])
    report = align_candidate_heights(*args)
    assert not report['applied'], report
    assert args[0] == before


def test_integer_and_float_depth_have_equivalent_units():
    """Aligned RealSense millimetres and ROS float metres give the same correction."""
    args = list(scene())
    args[2] = np.round(args[2] * 1000).astype(np.uint16)
    report = align_candidate_heights(*args)
    assert report['applied']
    assert args[0][0]['floor_correction_m'] == pytest.approx(0.011)


def test_selection_sends_aligned_height_to_planner(monkeypatch):
    """Apply floor alignment before IK, retain it through navigation-only selection."""
    from cleanup_perception.candidate_selection import select_body_sample

    _, mask, depth, k, rotation, translation, floor = scene()
    wrist = SimpleNamespace(rotation=SimpleNamespace(x=0., y=0., z=0., w=1.),
                            translation=SimpleNamespace(x=0., y=0., z=0.))
    calibration = SimpleNamespace(rotation=rotation, translation=translation)

    def planner(*args, **kwargs):
        rows = [[float(v) for v in line.split()] for line in kwargs['input'].splitlines()]
        assert rows
        assert all(abs(row[2] + 0.069) < 1e-9 for row in rows)
        assert all(row[3] == floor for row in rows)
        return SimpleNamespace(stdout='\n'.join(json.dumps({'feasible': False}) for _ in rows))

    monkeypatch.setattr('cleanup_perception.grasp_candidates.subprocess.run', planner)
    selected, report = select_body_sample(
        mask, depth, k, wrist, calibration, .03, floor, [0.] * 4, 'unused', align_floor=True)
    assert report['floor_alignment']['applied']
    assert not selected['plan']['feasible']  # Correcting height alone never permits execution.
    assert selected['observed_link1'][2] == pytest.approx(-0.069)
