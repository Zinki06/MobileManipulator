"""Ensure adding a manipulation checker cannot change ordinary navigation."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription
import yaml


def test_launch_selects_explicit_ordinary_checker(tmp_path, monkeypatch):
    """Resolve launch substitutions without starting any robot nodes."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'launch_logs'))
    package = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location(
        'aruco_launch_test', package / 'launch' / 'aruco_launcher.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    installed_lookup = module.get_package_share_directory
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda name: str(package) if name == 'aruco_localizer'
                        else installed_lookup(name))
    description = module.generate_launch_description()
    group = next(action for action in description.entities if isinstance(action, GroupAction))
    include = next(action for action in group.get_sub_entities()
                   if isinstance(action, IncludeLaunchDescription))
    params = dict(include.launch_arguments)['params_file'].perform(LaunchContext())
    settings = yaml.safe_load(Path(params).read_text())['bt_navigator']['ros__parameters']
    for field in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
        root = ET.parse(settings[field]).getroot()
        assert not root.findall('.//Spin')
        assert not root.findall('.//BackUp')
        followers = root.findall('.//FollowPath')
        assert followers
        for follower in followers:
            assert follower.get('controller_id') == 'StationPath'
            assert follower.get('goal_checker_id') == 'general_goal_checker'


def test_both_clients_use_shared_execution():
    """Keep patrol and cleanup from silently bypassing common motion supervision."""
    workspace = Path(__file__).parents[3]
    for path in [workspace / 'src/aruco_localizer/launch/aruco_launcher.launch.py',
                 workspace / 'src/project_bringup/launch/project.launch.py']:
        source = path.read_text()
        assert "'navigate_to_pose_action': '/motion/navigate_to_pose'" in source
        assert "'spin_action': '/motion/spin'" in source
