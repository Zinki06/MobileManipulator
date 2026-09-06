"""Check executable speeds and bounded safety waits independently of ROS."""

import math

import pytest

from robot_motion.policy import ProgressBudget, guard_allows_motion, spin_velocity, wrap


def test_turn_survives_integer_hardware_conversion():
    """Every nonzero target velocity survives OpenCR's centi-unit truncation."""
    for error in [-math.pi, -0.026, 0.026, math.pi]:
        speed = spin_velocity(error)
        assert int(speed * 100) != 0
        assert speed * error > 0
        assert abs(speed) <= 0.30
    assert spin_velocity(0.024) == 0
    with pytest.raises(ValueError):
        spin_velocity(float('nan'))


def test_safety_wait_does_not_consume_turn_time():
    """The logged 28-second stop must not expire a 30-second active turn."""
    budget = ProgressBudget(30.0, 30.0)
    assert not budget.update(2.0, False)
    assert not budget.update(28.0, True)
    assert not budget.update(6.0, False)
    assert budget.active == 8.0
    assert budget.blocked == 0.0
    assert budget.update(30.0, True) == 'safety wait expired'


def test_faults_never_authorize_motion():
    """Only readiness and an idle command watchdog allow new motion."""
    assert guard_allows_motion('READY')
    assert guard_allows_motion('BLOCKED: command timeout')
    assert not guard_allows_motion('BLOCKED: collision monitor stopped motion')
    assert not guard_allows_motion('FAULT: map reset')
    assert not guard_allows_motion('')
    assert wrap(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)
