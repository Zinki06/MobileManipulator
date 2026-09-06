"""Hardware-aware, ROS-independent motion policies."""

import math


def wrap(angle):
    """Normalize an angle without discarding accumulated multi-turn progress."""
    return math.atan2(math.sin(angle), math.cos(angle))


def spin_velocity(error, tolerance=0.025, maximum=0.30, gain=1.0, acceleration=0.4):
    """Use executable nonzero angular speeds; never boost collision-filtered output."""
    if not all(math.isfinite(v) for v in (error, maximum, gain, acceleration)) or \
            maximum < 0.08 or maximum > 1.82 or gain <= 0 or acceleration <= 0:
        raise ValueError('Non-finite rotation error')
    if abs(error) <= tolerance:
        return 0.0
    braking = math.sqrt(2 * acceleration * max(0., abs(error) - tolerance))
    return math.copysign(min(maximum, max(0.08, min(gain * abs(error), braking))), error)


def guard_allows_motion(status):
    """Allow an idle command timeout before starting a motion."""
    return status in ('READY', 'BLOCKED: command timeout')


class ProgressBudget:
    """Separate active movement time from bounded consecutive safety waits."""

    def __init__(self, active_limit, wait_limit):
        self.active_limit = active_limit
        self.wait_limit = wait_limit
        self.active = 0.0
        self.blocked = 0.0

    def update(self, dt, blocked):
        """Return a reason when a limit expires; never silently extend forever."""
        if blocked:
            self.blocked += dt
        else:
            self.blocked = 0.0
            self.active += dt
        if self.blocked >= self.wait_limit:
            return 'safety wait expired'
        if self.active >= self.active_limit:
            return 'active motion deadline expired'
        return ''
