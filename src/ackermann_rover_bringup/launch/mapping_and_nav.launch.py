import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('ackermann_rover_bringup')
    # pkg_explore = get_package_share_directory('explore_lite')
    
    # 1. Bringup (Bridge, URDF, TF)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'bringup.launch.py')),
        launch_arguments={'world_name': LaunchConfiguration('map_name')}.items()
    )
    
    # 2. Odometry Script
    odom_node = Node(
        package='ackermann_rover_bringup',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 2.5 Tiền xử lý cảm biến (LiDAR và Camera)
    sensor_processing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'sensor_processing.launch.py'))
    )
    
    # 2.6 Hệ thống nhận thức (Semantic Fusion & YOLO)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'perception.launch.py'))
    )
    
    # 3. SLAM Toolbox (Vẽ map + cung cấp map->odom TF cho định vị)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'slam.launch.py'))
    )
    
    # 4. Nav2 (Chỉ chạy Navigation, không chạy AMCL)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'nav2.launch.py'))
    )
    
    # 5. M-Explore (Đã tắt theo yêu cầu để chỉ dùng điều hướng thủ công)
    # explore_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_explore, 'launch', 'explore.launch.py'))
    # )
    
    # 6. Mission Manager Node
    mission_manager_node = Node(
        package='ackermann_rover_bringup',
        executable='mission_manager.py',
        name='mission_manager',
        output='screen'
    )


    # 6.5 PX4 Bridge Node
    nav2_px4_bridge_node = Node(
        package='ackermann_rover_bringup',
        executable='nav2_px4_bridge.py',
        name='nav2_px4_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # 6.6 Path Tracker Node (vẽ lại đường xe đã đi qua)
    path_tracker_node = Node(
        package='ackermann_rover_bringup',
        executable='path_tracker.py',
        name='path_tracker',
        output='screen'
    )
    
    
    # 7. RViz2
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'rviz.launch.py'))
    )


    # ---------------------------------------------------------
    # LƯU BẢN ĐỒ ĐÚNG 1 LẦN SAU 90 GIÂY
    # ---------------------------------------------------------
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='nav_world',
        description='Tên của map khi lưu (ví dụ: nav_world, map1, map2)'
    )
    
    auto_save_posegraph = ExecuteProcess(
        cmd=[
            'bash', '-c',
            ['sleep 90; /home/tinvo/rover_ackermann/src/ackermann_rover_bringup/scripts/save_posegraph.sh ', LaunchConfiguration('map_name')]
        ],
        output='screen'
    )

    # Đợi 90s rồi tự động gọi file save_map.sh (Chỉ chạy 1 lần)
    auto_save_map_2d = ExecuteProcess(
        cmd=[
            'bash', '-c',
            ['sleep 90; /home/tinvo/rover_ackermann/src/ackermann_rover_bringup/scripts/save_map.sh ', LaunchConfiguration('map_name'), '_map_2d']
        ],
        output='screen'
    )

    delayed_nav2 = TimerAction(period=3.0, actions=[nav2_launch])
    delayed_mission_manager = TimerAction(period=6.0, actions=[mission_manager_node])

    return LaunchDescription([
        map_name_arg,
        bringup_launch,
        odom_node,
        sensor_processing_launch,
        perception_launch,
        slam_launch,
        rviz_launch,
        delayed_nav2,
        delayed_mission_manager,
        nav2_px4_bridge_node,
        path_tracker_node,
        # auto_save_posegraph
      #  auto_save_map_2d
    ])
