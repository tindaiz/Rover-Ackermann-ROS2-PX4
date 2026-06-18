import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Khởi chạy node tiền xử lý dữ liệu
    sensor_preprocessor_node = Node(
        package='ackermann_rover_bringup',
        executable='sensor_preprocessor.py',
        name='sensor_preprocessor_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    collision_detector_node = Node(
        package='ackermann_rover_bringup',
        executable='collision_detector_node.py',
        name='collision_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        sensor_preprocessor_node,
        collision_detector_node
    ])
