"""Resolve one consistent motion profile for Nav2, safety and hardware limits."""

from copy import deepcopy
import math
from pathlib import Path
import tempfile

from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import yaml


def merge_parameters(base, overlay):
    """Merge nested ROS parameter maps without losing unrelated safety settings."""
    result = deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_parameters(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def validate_profile(profile):
    """Reject inconsistent speed caps and insufficient modeled stopping horizons."""
    guard = profile['motion_guard']['ros__parameters']
    drive = profile['diff_drive_controller']['ros__parameters']
    smoother = profile['velocity_smoother']['ros__parameters']
    model = profile['performance_safety']['ros__parameters']
    speed, angular = guard['max_linear_velocity'], guard['max_angular_velocity']
    values = [speed, angular, *model.values()]
    if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in values):
        raise ValueError('Motion profile requires finite positive limits')
    if speed > 0.26 or angular > 1.82:
        raise ValueError('Motion profile exceeds Waffle Pi hardware specification')
    if (drive['linear.x.max_velocity'] != speed or smoother['max_velocity'][0] != speed or
            drive['angular.z.max_velocity'] != angular or
            smoother['max_velocity'][2] != angular):
        raise ValueError('Hardware, guard and smoother velocity caps disagree')
    executor = profile['motion_executor']['ros__parameters']
    if executor['spin_max_velocity'] > angular:
        raise ValueError('Rotation velocity exceeds guard cap')
    for axis, index, field in [('linear', 0, 'linear.x'), ('angular', 2, 'angular.z')]:
        acceleration = guard[axis + '_acceleration']
        if not math.isfinite(acceleration) or acceleration <= 0 or not (
                drive[field + '.max_acceleration'] == acceleration ==
                smoother['max_accel'][index]):
            raise ValueError('Hardware, guard and smoother accelerations disagree')
    collision = profile['collision_monitor']['ros__parameters']
    if (model['reaction_seconds'] < collision['source_timeout'] + 0.1 or
            model['collision_horizon_seconds'] !=
            collision['ApproachFootprint']['time_before_collision']):
        raise ValueError('Stopping model disagrees with collision monitor timing')
    controllers = profile['controller_server']['ros__parameters']
    if any(controllers[name]['desired_linear_vel'] > speed
           for name in ('StationPath', 'ApproachPath')):
        raise ValueError('Planner velocity exceeds guard cap')
    stopping = (speed * model['reaction_seconds'] + speed**2 /
                (2 * model['braking_deceleration']) + model['margin_metres'])
    if stopping > speed * model['collision_horizon_seconds']:
        raise ValueError('Collision horizon is shorter than modeled stopping distance')
    if min(-drive['linear.x.min_acceleration'], -smoother['max_decel'][0]) < \
            model['braking_deceleration']:
        raise ValueError('Hardware braking is weaker than the stopping model')
    return stopping


class PerformanceParameters(Substitution):
    """Write a merged temporary parameter file; never modify a vendor source."""

    def __init__(self, source, profile):
        super().__init__()
        self.source = normalize_to_list_of_substitutions(source)
        self.profile = normalize_to_list_of_substitutions(profile)

    def describe(self):
        """Describe the launch substitution."""
        return 'Validated performance parameter overlay'

    def perform(self, context):
        """Resolve the selected profile and overlay it on the source YAML."""
        base = yaml.safe_load(Path(perform_substitutions(context, self.source)).read_text())
        profile = yaml.safe_load(Path(perform_substitutions(context, self.profile)).read_text())
        validate_profile(profile)
        with tempfile.NamedTemporaryFile(mode='w', prefix='motion_profile_',
                                         suffix='.yaml', delete=False) as output:
            yaml.safe_dump(merge_parameters(base, profile), output)
            return output.name
