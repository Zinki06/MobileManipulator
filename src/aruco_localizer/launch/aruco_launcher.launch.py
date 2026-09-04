import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 패키지 이름 및 경로 설정
    pkg_name = 'aruco_localizer'
    nav2_pkg_name = 'nav2_bringup'

    pkg_dir = get_package_share_directory(pkg_name)
    nav2_dir = get_package_share_directory(nav2_pkg_name)

    # Nav2 파라미터 파일 경로 설정
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    marker_yaml_file = os.path.join(pkg_dir, 'map', 'new_map_markers.yaml')
    map_yaml_file = os.path.join(pkg_dir, 'map', 'new_map.yaml')
    route_yaml_file = os.path.join(pkg_dir, 'config', 'routes.yaml')

    debug_output_dir = LaunchConfiguration('debug_output_dir')
    declared_arguments = [
        DeclareLaunchArgument(
            'debug_output_dir',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'turtlebot3_ws', 'nav_debug'
            ]),
            description='Directory for per-run navigation logs and snapshots.',
        ),
    ]

    # 1. ArUco Localizer 노드
    aruco_localizer_node = Node(
        package=pkg_name,
        executable='aruco_localizer_node',
        name='aruco_localizer',
        output='screen',
        parameters=[{'marker_yaml_path': marker_yaml_file}],
    )

    # 2. Map Marker Publisher 노드
    map_marker_publisher_node = Node(
        package=pkg_name,
        executable='map_marker_publisher_node',
        name='map_marker_publisher',
        output='screen',
        parameters=[{
            'map_yaml_path': map_yaml_file,
            'marker_yaml_path': marker_yaml_file,
        }],
    )

    # 3. Nav2 Navigation 스택
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true',
            'use_composition': 'False',  # 개별 프로세스 실행 및 로그 출력 활성화
        }.items(),
    )

    # 4. ArUco 직선 경로 네비게이터 노드
    aruco_waypoint_navigator_node = Node(
        package=pkg_name,
        executable='aruco_waypoint_navigator_node',
        name='aruco_waypoint_navigator',
        output='screen',
        parameters=[{'route_yaml_path': route_yaml_file}],
    )

    # 5. 카메라 스냅샷/경로 이벤트 기록은 제어 노드와 분리
    navigation_debug_recorder_node = Node(
        package=pkg_name,
        executable='navigation_debug_recorder_node',
        name='navigation_debug_recorder',
        output='screen',
        parameters=[{'output_directory': debug_output_dir}],
    )

    return LaunchDescription(declared_arguments + [
        aruco_localizer_node,
        map_marker_publisher_node,
        aruco_waypoint_navigator_node,
        navigation_debug_recorder_node,
        nav2_navigation
    ])
