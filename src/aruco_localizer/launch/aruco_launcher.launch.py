import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # 1. ArUco Localizer 노드
    aruco_localizer_node = Node(
        package=pkg_name,
        executable='aruco_localizer_node',
        name='aruco_localizer',
        output='screen',
    )

    # 2. Map Marker Publisher 노드
    map_marker_publisher_node = Node(
        package=pkg_name,
        executable='map_marker_publisher_node',
        name='map_marker_publisher',
        output='screen',
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
    )

    return LaunchDescription([
        aruco_localizer_node,
        map_marker_publisher_node,
        aruco_waypoint_navigator_node,
        nav2_navigation
    ])