from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    segmentation_node = Node(
        package='turtlebot3_pick_place',
        executable='segmentation_node',
        name='segmentation_node',
        output='screen'
    )
    
    realsense_tf_node = Node(
        package='realsense_bringup',
        executable='realsense_tf_node',
        name='realsense_tf_node',
        output='screen'
    )

    return LaunchDescription([
        segmentation_node,
        realsense_tf_node
    ])
