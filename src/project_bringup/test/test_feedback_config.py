# Copyright 2026 TurtleBot3 Project Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Validate project feedback overrides without starting hardware."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
import yaml


def test_feedback_overrides_preserve_vendor_file(tmp_path, monkeypatch):
    """Feedback and speed overrides are applied to a generated file only."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    path = Path(__file__).parents[2] / 'manipulation_bringup/launch/hardware.launch.py'
    spec = importlib.util.spec_from_file_location('feedback_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = Path(get_package_share_directory('turtlebot3_manipulation_bringup')) / \
        'config/hardware_controller_manager.yaml'
    before = original.read_bytes()
    rewritten = module.controller_parameters(str(original)).perform(LaunchContext())
    config = yaml.safe_load(Path(rewritten).read_text())
    assert original.read_bytes() == before
    drive = config['diff_drive_controller']['ros__parameters']
    assert drive['open_loop'] is False
    assert drive['linear.x.max_velocity'] == 0.10
    assert drive['angular.z.max_velocity'] == 0.35
    assert config['arm_controller']['ros__parameters']['open_loop_control'] is False
    grip = config['gripper_controller']['ros__parameters']
    assert grip['goal_tolerance'] == 0.001
    assert grip['allow_stalling'] is True
    assert grip['stall_timeout'] == 0.7


def test_project_connects_calibrated_candidate_pipeline(tmp_path, monkeypatch):
    """Resolve real launch parameters without executing any node or robot action."""
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters

    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    path = Path(__file__).parents[1] / 'launch' / 'project.launch.py'
    spec = importlib.util.spec_from_file_location('project_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    parameters = {}
    for action in description.entities:
        if not isinstance(action, Node):
            continue
        config = {}
        for item in evaluate_parameters(context, action._Node__parameters):
            if isinstance(item, dict):
                config.update(item)
            else:
                node_name = {'pick_and_place': 'pick_and_place_action_node',
                             'cleanup_task_manager_node': 'cleanup_task_manager'}.get(
                                 action.node_executable, action.node_executable)
                config.update(yaml.safe_load(Path(item).read_text()).get(
                    node_name, {}).get('ros__parameters', {}))
        parameters[action.node_executable] = config
    perception = parameters['scan_perception_node']
    pick = parameters['pick_and_place']
    assert perception['use_grasp_camera_calibration']
    assert perception['use_body_candidates']
    assert perception['empty_scene_frames'] == 2
    assert pick['startup_park'] is False
    assert pick['reached_joint_tolerance'] == 0.015
    assert parameters['cleanup_task_manager_node']['scan_settle_seconds'] == 0.10
    assert pick['use_candidate_grasp']
    assert pick['grasp_forward_offset'] == perception['grasp_forward_offset'] == 0.03
    assert pick['grasp_min_body_depth'] == perception['grasp_min_body_depth'] == 0.004
    assert pick['approach_error_margin'] == 0.0
    nav_config = yaml.safe_load((Path(get_package_share_directory('aruco_localizer')) /
                                 'config/nav2_params.yaml').read_text())
    assert pick['approach_goal_tolerance'] == nav_config['controller_server']['ros__parameters'][
        'manipulation_goal_checker']['xy_goal_tolerance']
    assert parameters['cleanup_task_manager_node']['require_candidate_grasp']
    manager = parameters['cleanup_task_manager_node']
    assert pick['target_max_age'] == manager['target_max_age'] == 6.0
    assert manager['verify_pick_with_camera'] is False
    config = yaml.safe_load(Path(parameters['cleanup_task_manager_node'][
        'task_config_path']).read_text())['cleanup']['scan']
    assert config['turns'] == 8
    assert abs(config['turn_angle']) < 0.786


def test_hardware_shutdown_wrapper_is_connected(tmp_path, monkeypatch):
    """Use the shutdown wrapper without executing launch actions or hardware."""
    from launch_ros.actions import Node

    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    path = Path(__file__).parents[2] / 'manipulation_bringup/launch/hardware.launch.py'
    spec = importlib.util.spec_from_file_location('shutdown_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    nodes = [action for action in module.generate_launch_description().entities
             if isinstance(action, Node)]
    assert sum(n.node_executable == 'controlled_hardware' for n in nodes) == 1
    assert not any(n.node_executable == 'ros2_control_node' for n in nodes)
