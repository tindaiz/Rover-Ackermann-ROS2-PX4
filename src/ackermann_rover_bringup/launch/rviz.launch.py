import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'ackermann_rover_bringup'
    pkg_share = get_package_share_directory(pkg_name)
    
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'rover_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])
