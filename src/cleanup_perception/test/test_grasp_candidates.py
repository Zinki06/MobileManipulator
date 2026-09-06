"""Check candidate filtering and reachability selection without hardware."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from cleanup_perception.grasp_candidates import body_candidates, choose_candidate


def scene():
    """Return an elongated surface with known metric width and camera transform."""
    mask = np.zeros((160, 300), dtype=np.uint8)
    mask[50:110, 30:270] = 1
    depth = np.full(mask.shape, 300, dtype=np.uint16)
    k = np.array([[500., 0., 150.], [0., 500., 80.], [0., 0., 1.]])
    rotation = np.diag([1., -1., -1.])
    return mask, depth, k, rotation, np.array([0.3, 0., 0.23])


def test_selects_body_interior_with_supported_width():
    """Retain multiple central body sections while avoiding narrow object ends."""
    candidates = body_candidates(*scene())
    assert len(candidates) >= 3
    assert max(c['pixel'][0] for c in candidates) - min(c['pixel'][0] for c in candidates) > 30
    for candidate in candidates:
        assert 0.029 < candidate['width_m'] < 0.037
        assert candidate['end_margin_m'] >= 0.02
        assert abs(candidate['depth_m'] - 0.3) < 1e-6


def test_rejects_missing_depth_and_excessive_width():
    """No geometric score may rescue an unsupported or too-wide surface."""
    args = list(scene())
    args[1][:] = 0
    assert body_candidates(*args) == []
    args = list(scene())
    args[0][15:145, 30:270] = 1
    assert body_candidates(*args) == []


def test_offset_is_explicit_and_bounded():
    """A caller cannot introduce an unbounded experimental calibration offset."""
    with pytest.raises(ValueError):
        body_candidates(*scene(), forward_offset=0.04)
    with pytest.raises(ValueError):
        body_candidates(*scene(), forward_offset=float('nan'))


def test_unreachable_candidates_cannot_win(monkeypatch):
    """Select the sole reachable candidate regardless of its geometric rank."""
    candidates = body_candidates(*scene())
    plans = [{'feasible': False} for _ in candidates]
    plans[-1] = {'feasible': True, 'body_depth': 0.006, 'pitch_degrees': -35.}
    monkeypatch.setattr('cleanup_perception.grasp_candidates.subprocess.run',
                        lambda *args, **kwargs: SimpleNamespace(
                            stdout='\n'.join(json.dumps(p) for p in plans)))
    selected, evaluated = choose_candidate(candidates, -0.101, [0., 0., 0., 0.], 'unused')
    assert selected['pixel'] == candidates[-1]['pixel']
    assert len(evaluated) == len(candidates)


def test_no_reachable_candidate_requires_reapproach(monkeypatch):
    """Never fall back to the old centroid when all complete paths fail."""
    candidates = body_candidates(*scene())
    monkeypatch.setattr('cleanup_perception.grasp_candidates.subprocess.run',
                        lambda *args, **kwargs: SimpleNamespace(
                            stdout='{"feasible":false}\n' * len(candidates)))
    with pytest.raises(ValueError, match='reapproach'):
        choose_candidate(candidates, -0.101, [0., 0., 0., 0.], 'unused')


def test_recording_failure_keeps_completed_hover():
    """An image timeout must not report the already completed motion as failed."""
    from cleanup_perception.candidate_hover_trial import HoverTrial

    trial = HoverTrial.__new__(HoverTrial)
    trial.report = {'hover_complete': True}
    trial.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(warning=lambda msg: None))

    def timeout():
        raise RuntimeError('camera delayed')

    trial._save_hover_evidence = timeout
    trial._record_hover_evidence()
    assert trial.report['hover_complete']
    assert trial.report['evidence_error'] == 'camera delayed'
    assert 'error' not in trial.report


@pytest.mark.parametrize('feasible', [True, False])
def test_automatic_selection_uses_trial_geometry(monkeypatch, feasible):
    """Automatic selection preserves the correction and cannot invent feasibility."""
    from cleanup_perception.candidate_selection import select_body_sample

    mask, depth, k, rotation, translation = scene()
    wrist = SimpleNamespace(rotation=SimpleNamespace(x=0., y=0., z=0., w=1.),
                            translation=SimpleNamespace(x=0., y=0., z=0.))
    calibration = SimpleNamespace(rotation=rotation, translation=translation)
    candidates = body_candidates(mask, depth, k, rotation, translation, 0.03)
    plans = [{'feasible': False} for _ in candidates]
    if feasible:
        plans[0] = {'feasible': True, 'body_depth': 0.008, 'pitch_degrees': -20.}
    monkeypatch.setattr('cleanup_perception.grasp_candidates.subprocess.run',
                        lambda *args, **kwargs: SimpleNamespace(
                            stdout='\n'.join(json.dumps(p) for p in plans)))
    selected, report = select_body_sample(
        mask, depth, k, wrist, calibration, 0.03, -0.101, [0.] * 4, 'unused')
    assert selected['plan']['feasible'] == feasible
    assert np.allclose(np.subtract(selected['target_link1'], selected['observed_link1']),
                       [0.03, 0., 0.])
    if feasible:
        assert selected['pixel'] == candidates[0]['pixel']
    else:
        assert all(not c['plan']['feasible'] for c in report['evaluated'])
        assert selected['body_percentile'] == 50
