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

"""Start the original TurtleBot3 hardware and standard controllers."""

import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml
from robot_motion.performance import PerformanceParameters


class FeedbackParameters(RewrittenYaml):
    """Apply gripper completion settings to the generated configuration copy."""

    def perform(self, context):
        """Preserve drive rewrites and require millimetre-scale finger completion."""
        path = super().perform(context)
        config = yaml.safe_load(Path(path).read_text())
        config['gripper_controller']['ros__parameters'].update({
            'goal_tolerance': 0.001,
            'allow_stalling': True,
            'stall_timeout': 0.7,
        })
        Path(path).write_text(yaml.safe_dump(config))
        return path


def controller_parameters(source):
    """Use measured feedback for the drive and arm controllers."""
    return FeedbackParameters(
        source_file=source, root_key='', convert_types=True,
        param_rewrites={
            'open_loop': 'false',
            'open_loop_control': 'false',
            'linear.x.max_velocity': '0.10',
            'linear.x.min_velocity': '-0.05',
            'linear.x.max_acceleration': '0.15',
            'angular.z.max_velocity': '0.35',
            'angular.z.min_velocity': '-0.35',
            'angular.z.max_acceleration': '0.4',
            'angular.z.min_acceleration': '-0.8',
        })


def require_hardware_mode(context):
    """Never silently launch real hardware for a requested simulator session."""
    if LaunchConfiguration('use_sim').perform(context).lower() == 'true':
        raise RuntimeError(
            'Feedback robot launch does not start Gazebo; use_sim=true is rejected.')
    return []


def generate_launch_description():
    """Start the original OpenCR driver and standard ROS controllers."""
    description = get_package_share_directory('turtlebot3_manipulation_description')
    bringup = get_package_share_directory('turtlebot3_manipulation_bringup')
    robot = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(description, 'urdf', 'turtlebot3_manipulation.urdf.xacro'),
        ' prefix:=\"\" use_sim:=false fake_sensor_commands:=false use_fake_hardware:=',
        LaunchConfiguration('use_fake_hardware'),
    ]), value_type=str)
    controller = Node(
        package='robot_motion', executable='controlled_hardware', output='screen',
        # Allow release waits plus the child's 10s graceful / 5s forced cleanup.
        sigterm_timeout='30', sigkill_timeout='10',
        parameters=[{'robot_description': robot}, PerformanceParameters(controller_parameters(
            os.path.join(bringup, 'config', 'hardware_controller_manager.yaml')),
            LaunchConfiguration('performance_config_file'))],
        remappings=[('~/cmd_vel_unstamped', '/cmd_vel'), ('~/odom', '/odom')],
    )
    joints = Node(package='controller_manager', executable='spawner',
                  arguments=['joint_state_broadcaster', '-c', '/controller_manager'])
    others = [Node(package='controller_manager', executable='spawner',
                   arguments=[name, '-c', '/controller_manager'])
              for name in ('diff_drive_controller', 'imu_broadcaster',
                           'arm_controller', 'gripper_controller')]
    lidar_package, lidar_file = ('hls_lfcd_lds_driver', 'hlds_laser.launch.py') \
        if os.environ.get('LDS_MODEL') == 'LDS-01' else ('ld08_driver', 'ld08.launch.py')
    lidar = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory(lidar_package), 'launch', lidar_file)),
        launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items())
    return LaunchDescription([
        DeclareLaunchArgument('performance_config_file', default_value=os.path.join(
            get_package_share_directory('aruco_localizer'), 'config', 'performance.yaml')),
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        OpaqueFunction(function=require_hardware_mode),
        RegisterEventHandler(OnProcessExit(target_action=joints, on_exit=others)),
        RegisterEventHandler(OnProcessExit(target_action=others[2], on_exit=[TimerAction(
            period=1.0,
            actions=[ExecuteProcess(cmd=[
                'ros2', 'action', 'send_goal',
                '/arm_controller/follow_joint_trajectory',
                'control_msgs/action/FollowJointTrajectory',
                "{trajectory: {joint_names: [joint1, joint2, joint3, joint4], points: "
                "[{positions: [0.0, -0.523, -0.523, 1.5707], time_from_start: {sec: 3}}]}}",
                '--feedback',
            ], output='both')])])),
        controller,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot}], output='screen'),
        joints,
        lidar,
    ])
