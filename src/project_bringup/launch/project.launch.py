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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include_launch(package_name, launch_file, condition, launch_arguments=None):
    """Create a conditional include for an installed Python launch file."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(package_name), 'launch', launch_file]
            )
        ),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    """Start the complete hardware, perception, navigation, and grasping stack."""
    start_robot = LaunchConfiguration('start_robot')
    start_realsense = LaunchConfiguration('start_realsense')
    start_segmentation = LaunchConfiguration('start_segmentation')
    start_navigation = LaunchConfiguration('start_navigation')
    start_pick_and_place = LaunchConfiguration('start_pick_and_place')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    declared_arguments = [
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
            default_value='true',
            description='Start the YOLO and EfficientTAM segmentation node.',
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
            'use_fake_hardware',
            default_value='false',
            description='Use fake ros2_control hardware for the base and arm.',
        ),
    ]

    actions = [
        include_launch(
            'turtlebot3_manipulation_bringup',
            'hardware.launch.py',
            IfCondition(start_robot),
            {
                'start_rviz': 'false',
                'use_fake_hardware': use_fake_hardware,
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
        include_launch(
            'aruco_localizer',
            'aruco_launcher.launch.py',
            IfCondition(start_navigation),
        ),
        Node(
            package='pick_and_place',
            executable='pick_and_place',
            name='pick_and_place_action_node',
            output='screen',
            condition=IfCondition(start_pick_and_place),
        ),
    ]

    return LaunchDescription(declared_arguments + actions)
