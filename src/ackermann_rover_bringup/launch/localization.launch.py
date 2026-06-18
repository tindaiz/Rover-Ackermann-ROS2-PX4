import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'ackermann_rover_bringup'
    pkg_share = get_package_share_directory(pkg_name)

    ekf_global_config  = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    default_map_path   = os.path.join(pkg_share, 'maps', 'nav.yaml')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Full path to map yaml file to load')

    params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=default_params_file,
        description='Full path to the ROS2 parameters file')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')

    # ---------------------------------------------------------------
    # ekf_global_node: fuse odom + GPS → publish TF: map -> odom
    # (ekf_local_node đã bị XÓA vì nó cạnh tranh với odom_to_tf.py
    #  cho TF odom -> base_link gây ra lag/giật)
    # ---------------------------------------------------------------
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config],
        remappings=[('odometry/filtered', 'odometry/filtered_global')]
    )

    # navsat_transform_node: chuyển GPS fix -> /odometry/gps cho ekf_global
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            'frequency': 30.0,
            'delay': 3.0,
            'magnetic_declination_radians': 0.0,
            'yaw_offset': 0.0,
            'zero_altitude': True,
            'broadcast_cartesian_transform': False,
            'publish_filtered_gps': True,
            'use_odometry_yaw': False,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('imu', '/imu'),
            ('gps/fix', '/gps/fix'),
            ('gps/filtered', '/gps/filtered'),
            ('odometry/gps', '/odometry/gps'),
            ('odometry/filtered', '/odometry/filtered_global')
        ]
    )

    # ---------------------------------------------------------------
    # map_server + AMCL: load bản đồ tĩnh, định vị bằng laser
    # AMCL KHÔNG phát TF (tf_broadcast: False trong nav2_params.yaml)
    # vì ekf_global_node đã lo việc đó
    # ---------------------------------------------------------------
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'yaml_filename': LaunchConfiguration('map'),
             'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        map_yaml_cmd,
        params_file_cmd,
        use_sim_time_cmd,
        # ekf_global_node,
        # navsat_transform_node,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])

