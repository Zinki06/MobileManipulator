import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import yaml

def generate_launch_description():
    # 1. Load Robot Description URDF
    robot_description_path = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_description'),
        'urdf',
        'turtlebot3_manipulation.urdf.xacro'
    )
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 2. Load Robot Description SRDF
    robot_description_semantic_path = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_moveit_config'),
        'config',
        'turtlebot3_manipulation.srdf'
    )
    with open(robot_description_semantic_path, 'r') as f:
        robot_description_semantic_config = f.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # 3. Load Kinematics YAML
    kinematics_yaml_path = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_moveit_config'),
        'config',
        'kinematics.yaml'
    )
    with open(kinematics_yaml_path, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)

    # 4. Perception Server Node (Python script)
    perception_node = Node(
        package='turtlebot3_pick_place',
        executable='yolo_sam2_perception_server.py',
        name='yolo_sam2_perception_server',
        output='screen'
    )
    
    # 5. Main C++ Pick & Place Orchestrator Node
    orchestrator_node = Node(
        package='turtlebot3_pick_place',
        executable='pick_place_orchestrator',
        name='pick_place_orchestrator',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml
        ]
    )

    return LaunchDescription([
        perception_node,
        orchestrator_node
    ])
