import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('ackermann_rover_bringup')
    
    # 1. Bringup (Bridge, URDF, TF)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'bringup.launch.py'))
    )
    
    # 2. Odometry Script
    odom_node = Node(
        package='ackermann_rover_bringup',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen'
    )
    
    # 3. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'slam.launch.py'))
    )
    
    # 4. RViz2
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'rviz.launch.py'))
    )

    return LaunchDescription([
        bringup_launch,
        odom_node,
        slam_launch,
        rviz_launch
    ])
