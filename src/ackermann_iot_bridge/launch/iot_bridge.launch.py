import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ackermann_iot_bridge',
            executable='system_health_node',
            name='system_health_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='ackermann_iot_bridge',
            executable='telemetry_aggregator_node',
            name='telemetry_aggregator_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='ackermann_iot_bridge',
            executable='transport_manager_node',
            name='transport_manager_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
