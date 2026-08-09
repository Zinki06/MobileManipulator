import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    rs_launch_package = get_package_share_directory('realsense2_camera')
    
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rs_launch_package, 'launch', 'rs_launch.py')
        ),
        launch_arguments = {
            'align_depth.enable': 'true',
            'camera_name': 'camera',
            'pointcloud.enable': 'false',

            # FPS를 0이 아닌 30으로 명확히 명시 (또는 이 두 줄을 아예 주석 처리)
            'rgb_camera.color_profile': '1280x720x15', 
            'depth_module.depth_profile': '848x480x15',
            
            # FPS를 낮추고 싶다면 아래처럼 설정
            # 'rgb_camera.color_profile': '1280x720x15',
            # 'depth_module.depth_profile': '848x480x15',
        }.items()
    )
    
    depth_to_pointcloud_node = Node(
        package='realsense_bringup',
        executable='realsense_tf_node',
        name='realsense_tf_node',
        output='screen',
    )

    return LaunchDescription([
        rs_launch,
        depth_to_pointcloud_node,
    ])