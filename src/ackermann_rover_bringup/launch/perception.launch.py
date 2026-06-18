from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('ackermann_rover_bringup')
    config_file = os.path.join(pkg_dir, 'config', 'fusion_params.yaml')

    return LaunchDescription([
        Node(
            package='ackermann_rover_bringup',
            executable='yolo_detector_node.py',
            name='yolo_detector_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='ackermann_rover_bringup',
            executable='semantic_fusion_node.py',
            name='semantic_fusion_node',
            output='screen',
            parameters=[config_file]
        )
    ])
