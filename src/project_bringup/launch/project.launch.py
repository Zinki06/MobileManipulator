#!/usr/bin/env python3
#
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

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, launch_file, condition, launch_arguments=None):
    """Include a launch file with specified arguments and condition."""
    arguments = launch_arguments or {}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package_name), 'launch', launch_file,
        ])),
        launch_arguments=arguments.items(),
        condition=condition,
    )


def read_env_value(path, key):
    """Read one literal value from a dotenv file without executing it."""
    try:
        lines = Path(path).expanduser().read_text(encoding='utf-8').splitlines()
    except (OSError, UnicodeError):
        return ''

    for line in lines:
        value = line.strip()
        if not value or value.startswith('#'):
            continue
        if value.startswith('export '):
            value = value[7:].lstrip()
        name, separator, raw_value = value.partition('=')
        if not separator or name.strip() != key:
            continue
        result = raw_value.strip()
        if (len(result) >= 2 and result[0] == result[-1] and
                result[0] in {'\'', '"'}):
            result = result[1:-1]
        return result
    return ''


def launch_cleanup_planner(context, model, env_file):
    """Start the planner with an API key from process env or local dotenv."""
    key = os.environ.get('GEMINI_API_KEY', '').strip()
    if not key:
        key = read_env_value(env_file.perform(context), 'GEMINI_API_KEY')
    return [Node(
        package='cleanup_planner',
        executable='gemini_planner_node',
        name='gemini_planner_node',
        output='screen',
        parameters=[{
            'planner_mode': 'gemini',
            'model': model,
        }],
        additional_env={'GEMINI_API_KEY': key},
    )]


def generate_launch_description():
    """Start the complete hardware, perception, navigation, and grasping stack."""
    start_robot = LaunchConfiguration('start_robot')
    start_realsense = LaunchConfiguration('start_realsense')
    start_segmentation = LaunchConfiguration('start_segmentation')
    start_cleanup_perception = LaunchConfiguration('start_cleanup_perception')
    start_cleanup_planner = LaunchConfiguration('start_cleanup_planner')
    start_navigation = LaunchConfiguration('start_navigation')
    start_pick_and_place = LaunchConfiguration('start_pick_and_place')
    start_cleanup_manager = LaunchConfiguration('start_cleanup_manager')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_sim = LaunchConfiguration('use_sim')
    spin_command_scale = LaunchConfiguration('spin_command_scale')
    cleanup_model_path = LaunchConfiguration('cleanup_model_path')
    cleanup_sam_model_path = LaunchConfiguration('cleanup_sam_model_path')
    gemini_model = LaunchConfiguration('gemini_model')
    gemini_env_file = LaunchConfiguration('gemini_env_file')

    profile = LaunchConfiguration('performance_config_file')
    declared_arguments = [
        DeclareLaunchArgument('performance_config_file', default_value=PathJoinSubstitution([
            FindPackageShare('aruco_localizer'), 'config', 'performance.yaml'])),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        ),
        DeclareLaunchArgument(
            'start_robot',
            default_value='true',
            description='Start the TurtleBot3 base, arm controllers, and lidar.',
        ),
        DeclareLaunchArgument(
            'start_realsense',
            default_value='true',
            description='Start the RealSense camera and 3D target processing.',
        ),
        DeclareLaunchArgument(
            'start_segmentation',
            default_value='false',
            description='Start the legacy YOLO and EfficientTAM tracker.',
        ),
        DeclareLaunchArgument(
            'start_cleanup_perception',
            default_value='true',
            description='Start request-driven YOLO/depth station perception.',
        ),
        DeclareLaunchArgument(
            'start_cleanup_planner',
            default_value='true',
            description='Start constrained Gemini cleanup planning.',
        ),
        DeclareLaunchArgument(
            'start_navigation',
            default_value='true',
            description='Start ArUco localization and the Nav2 stack.',
        ),
        DeclareLaunchArgument(
            'start_pick_and_place',
            default_value='true',
            description='Start the service-controlled pick-and-place node.',
        ),
        DeclareLaunchArgument(
            'start_cleanup_manager',
            default_value='true',
            description='Start the high-level cleanup mission state machine.',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake ros2_control hardware for the base and arm.',
        ),
        DeclareLaunchArgument(
            'spin_command_scale',
            default_value='1.0',
            description='Turn multiplier; keep 1.0 with encoder feedback odometry.',
        ),
        DeclareLaunchArgument(
            'cleanup_model_path',
            default_value='/home/user/turtlebot3_ws/src/segmentation/best.pt',
            description='YOLO weights used by request-driven cleanup perception.',
        ),
        DeclareLaunchArgument(
            'cleanup_sam_model_path',
            default_value='/home/user/turtlebot3_ws/src/segmentation/sam2_t.pt',
            description='SAM2 weights used for confirmed-object depth refinement.',
        ),
        DeclareLaunchArgument(
            'cleanup_camera_calibration_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cleanup_perception'), 'config', 'grasp_camera_20260906.yaml']),
            description='Perception-only camera extrinsics; nominal global TF remains unchanged.',
        ),
        DeclareLaunchArgument(
            'gemini_model',
            default_value='gemini-3.5-flash-lite',
            description='Gemini model used for bounded task planning.',
        ),
        DeclareLaunchArgument(
            'gemini_env_file',
            default_value='/home/user/turtlebot3_ws/.env',
            description='Local dotenv file containing GEMINI_API_KEY.',
        ),
        DeclareLaunchArgument('record_grasp_video', default_value='true',
                              description='Save approach/grasp video only for selected objects.'),
    ]

    actions = [
        include_launch(
            'project_bringup',
            'feedback_robot.launch.py',
            IfCondition(start_robot),
            {
                'start_rviz': 'false',
                'performance_config_file': profile,
                'use_fake_hardware': use_fake_hardware,
                'use_sim': use_sim,
            },
        ),
        include_launch(
            'realsense_bringup',
            'realsense_tf.launch.py',
            IfCondition(start_realsense),
        ),
        Node(
            package='segmentation',
            executable='segmentation_node',
            name='segmentation_node',
            output='screen',
            condition=IfCondition(start_segmentation),
        ),
        Node(
            package='cleanup_perception',
            executable='scan_perception_node',
            name='scan_perception_node',
            output='screen',
            condition=IfCondition(start_cleanup_perception),
            parameters=[profile, {
                'model_path': cleanup_model_path,
                'sam_model_path': cleanup_sam_model_path,
                'use_sam_refinement': True,
                'use_body_candidates': True,
                'grasp_forward_offset': 0.030,
            }, LaunchConfiguration('cleanup_camera_calibration_file')],
        ),
        OpaqueFunction(
            function=launch_cleanup_planner,
            args=[gemini_model, gemini_env_file],
            condition=IfCondition(start_cleanup_planner),
        ),
        include_launch(
            'aruco_localizer',
            'aruco_launcher.launch.py',
            IfCondition(start_navigation),
            {'spin_command_scale': spin_command_scale,
             'performance_config_file': profile,
             'record_grasp_video': LaunchConfiguration('record_grasp_video')},
        ),
        Node(
            package='pick_and_place',
            executable='pick_and_place',
            name='pick_and_place_action_node',
            output='screen',
            condition=IfCondition(start_pick_and_place),
            parameters=[profile, {
                'target_topic': '/cleanup/pick_target',
                'use_candidate_grasp': True, 'grasp_forward_offset': 0.030,
            }],
        ),
        Node(
            package='cleanup_task_manager',
            executable='cleanup_task_manager_node',
            name='cleanup_task_manager',
            output='screen',
            condition=IfCondition(start_cleanup_manager),
            parameters=[profile, {
                'task_config_path': PathJoinSubstitution([
                    FindPackageShare('cleanup_task_manager'),
                    'config',
                    'task_zones.yaml',
                ]),
                'spin_command_scale': spin_command_scale,
                'require_candidate_grasp': True,
                'navigate_to_pose_action': '/motion/navigate_to_pose',
                'spin_action': '/motion/spin',
            }],
        ),
    ]

    return LaunchDescription(declared_arguments + actions)
