"""Check the complete speed overlay and safety timing without launching a robot."""

from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from robot_motion.performance import merge_parameters, validate_profile
from robot_motion.policy import spin_velocity


CONFIG = Path(__file__).parents[2] / 'aruco_localizer/config'


@pytest.mark.parametrize('name', ['performance.yaml', 'performance_conservative.yaml'])
def test_profile_preserves_collision_checks_and_goal_tolerances(name):
    """Speed overlays retain footprint sensing and precision approach contracts."""
    profile = yaml.safe_load((CONFIG / name).read_text())
    assert validate_profile(profile) > 0
    before = yaml.safe_load((CONFIG / 'nav2_params.yaml').read_text())
    after = merge_parameters(before, profile)
    a, b = [x['controller_server']['ros__parameters'] for x in (before, after)]
    for key in a['goal_checker_plugins']:
        assert a[key] == b[key]
    for key in ('StationPath', 'ApproachPath'):
        assert b[key]['use_collision_detection']
        assert b[key]['use_regulated_linear_velocity_scaling']
    safety = yaml.safe_load((CONFIG / 'motion_safety.yaml').read_text())
    merged = merge_parameters(safety, profile)['collision_monitor']['ros__parameters']
    assert merged['StopFootprint'] == safety['collision_monitor']['ros__parameters'][
        'StopFootprint']
    assert merged['observation_sources'] == ['scan', 'depth_points']


def test_inconsistent_or_unsafe_profiles_rejected():
    """Reject caps silently defeated by another layer or an optimistic braking model."""
    baseline = yaml.safe_load((CONFIG / 'performance.yaml').read_text())
    for node, key, value in [('motion_guard', 'max_linear_velocity', .3),
                             ('velocity_smoother', 'max_velocity', [.1, 0., .6]),
                             ('diff_drive_controller', 'linear.x.min_acceleration', -.1),
                             ('performance_safety', 'reaction_seconds', 2.),
                             ('motion_guard', 'angular_acceleration', .4),
                             ('motion_executor', 'spin_max_velocity', .9)]:
        profile = deepcopy(baseline)
        profile[node]['ros__parameters'][key] = value
        with pytest.raises(ValueError):
            validate_profile(profile)


def test_fast_turn_reduces_time_without_losing_small_error_resolution():
    """A simple 50 Hz quantized encoder model validates direction and turn accuracy."""
    def simulate(maximum, gain, acceleration):
        angle, velocity = 0., 0.
        for step in range(1000):
            error = .785398 - angle
            command = spin_velocity(error, maximum=maximum, gain=gain,
                                    acceleration=acceleration)
            delta = max(-1.2 * .02, min(acceleration * .02, command - velocity))
            velocity += delta
            angle += int(velocity * 100) / 100 * .02
            if abs(error) <= .025 and abs(velocity) < .03:
                return step * .02, angle
        raise AssertionError('Turn did not converge')
    before, _ = simulate(.30, 1., .4)
    after, angle = simulate(.55, 2., .8)
    assert after < before * .75
    assert abs(angle - .785398) <= .03
