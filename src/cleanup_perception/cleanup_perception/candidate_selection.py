"""Shared candidate selection at the automatic perception boundary."""

import numpy as np

from cleanup_perception.grasp_candidates import body_candidates, choose_candidate
from cleanup_perception.grasp_candidates import NoFeasibleCandidate
from cleanup_perception.floor_alignment import align_candidate_heights


def select_body_sample(mask, depth, k, wrist, calibration, offset, floor, start, planner,
                       align_floor=False, min_body_depth=0.006):
    """Return a verified body candidate, or an explicit navigation-only candidate."""
    q, t = wrist.rotation, wrist.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    if not np.isclose(x*x + y*y + z*z + w*w, 1., atol=0.001):
        raise ValueError('Invalid wrist rotation')
    rw = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    rotation = rw @ calibration.rotation
    translation = np.array([t.x, t.y, t.z]) + rw @ calibration.translation
    candidates = body_candidates(mask, depth, k, rotation, translation, offset)
    floor_report = (align_candidate_heights(
        candidates, mask, depth, k, rotation, translation, floor) if align_floor else
        {'applied': False, 'reason': 'disabled'})
    try:
        selected, evaluated = choose_candidate(candidates, floor, start, planner, min_body_depth)
    except NoFeasibleCandidate as error:
        evaluated = error.evaluated
        # This target can guide base reapproach only; never execute it as a grasp.
        selected = min(evaluated, key=lambda c: abs(c['body_percentile'] - 50))
    return selected, {'selected': selected, 'evaluated': evaluated,
                      'rotation': rotation.tolist(), 'translation': translation.tolist(),
                      'floor_link1': floor, 'start_joints': list(start),
                      'floor_alignment': floor_report, 'min_body_depth_m': min_body_depth}
