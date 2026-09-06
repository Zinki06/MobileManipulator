"""Hardware-aware, ROS-independent motion policies."""

import math


def wrap(angle):
    """Normalize an angle without discarding accumulated multi-turn progress."""
    return math.atan2(math.sin(angle), math.cos(angle))


def spin_velocity(error, tolerance=0.025):
    """Use executable nonzero angular speeds; never boost collision-filtered output."""
    if not math.isfinite(error):
        raise ValueError('Non-finite rotation error')
    if abs(error) <= tolerance:
        return 0.0
    return math.copysign(min(0.30, max(0.08, abs(error))), error)


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
