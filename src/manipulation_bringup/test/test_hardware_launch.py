"""Verify independent hardware wiring without starting a controller or opening USB."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest


@pytest.mark.parametrize('fake', ['false', 'true'])
def test_expanded_model_selects_only_requested_hardware(tmp_path, monkeypatch, fake):
    """Expand real xacro and inspect parameters; no launch Node action is executed."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    path = Path(__file__).parents[1] / 'launch/hardware.launch.py'
    spec = importlib.util.spec_from_file_location('new_manipulation_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    reference = Path(get_package_share_directory('turtlebot3_manipulation_description'))
    original = reference / 'ros2_control/turtlebot3_manipulation_system.ros2_control.xacro'
    before = original.read_bytes()
    context = LaunchContext()
    context.launch_configurations['use_fake_hardware'] = fake
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    controller = next(a for a in description.entities if
                      isinstance(a, Node) and a.node_executable == 'controlled_hardware')
    values = evaluate_parameters(context, controller._Node__parameters)
    root = ET.fromstring(values[0]['robot_description'])
    plugins = [p.text for p in root.findall('./ros2_control/hardware/plugin')]
    assert plugins == ['fake_components/GenericSystem' if fake == 'true' else
                       'manipulation_hardware/OpenCRSystem']
    assert original.read_bytes() == before
    if fake == 'false':
        current = root.find('./ros2_control/hardware/param[@name="gripper_goal_current_raw"]')
        assert current.text == '80'
    assert root.find('./ros2_control/joint[@name="gripper_left_joint"]') is not None
