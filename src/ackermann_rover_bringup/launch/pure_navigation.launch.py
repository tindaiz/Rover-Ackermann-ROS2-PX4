import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('ackermann_rover_bringup')
    fusion_params_file = os.path.join(pkg_bringup, 'config', 'fusion_params.yaml')

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='nav_world',
        description='Tên của map để mô phỏng (dùng cho Gazebo)'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='nav.yaml',
        description='Tên file map yaml trong thư mục maps'
    )

    use_semantic_fusion_arg = DeclareLaunchArgument(
        'use_semantic_fusion',
        default_value='true',
        description='Bật/tắt YOLO + Semantic Fusion pipeline (tắt khi không có GPU)'
    )

    map_yaml_file = PathJoinSubstitution([
        pkg_bringup,
        'maps',
        LaunchConfiguration('map_file')
    ])

    # ─────────────────────────────────────────────────────────────────────────
    # 1. Bringup (Bridge, URDF, TF tĩnh)
    #    Cung cấp raw topics: /scan_raw, /imu, /gps/fix, /odom,
    #                         /camera/image_raw, /sonar_front/scan_raw, ...
    # ─────────────────────────────────────────────────────────────────────────
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'bringup.launch.py')),
        launch_arguments={'world_name': LaunchConfiguration('map_name')}.items()
    )

    # ─────────────────────────────────────────────────────────────────────────
    # 2. SENSOR FUSION – Tầng 1: Tiền xử lý tín hiệu cảm biến thô
    #    Input : /scan_raw, /sonar_front/scan_raw, /sonar_rear/scan_raw,
    #            /camera/image_raw
    #    Output: /scan (filtered), /sonar_front/scan, /sonar_rear/scan,
    #            /model_input/image_processed
    #    Phải chạy TRƯỚC localization và Nav2 để EKF & costmap nhận
    #    dữ liệu đã được lọc median (giảm spike, loại NaN/Inf).
    # ─────────────────────────────────────────────────────────────────────────
    sensor_preprocessor_node = Node(
        package='ackermann_rover_bringup',
        executable='sensor_preprocessor.py',
        name='sensor_preprocessor_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ─────────────────────────────────────────────────────────────────────────
    # 3. Odometry (odom → TF: odom → base_link)
    # ─────────────────────────────────────────────────────────────────────────
    odom_node = Node(
        package='ackermann_rover_bringup',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ─────────────────────────────────────────────────────────────────────────
    # 4. Localization (EKF Global + navsat_transform + AMCL + Map Server)
    #    EKF fuse: /odom (wheel) + /odometry/gps (GPS→ENU) + /imu
    #    → /odometry/filtered_global + TF: map → odom
    #    AMCL dùng /scan (đã filtered từ bước 2) để định vị trên bản đồ tĩnh.
    # ─────────────────────────────────────────────────────────────────────────
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'localization.launch.py')),
        launch_arguments={'map': map_yaml_file}.items()
    )

    # ─────────────────────────────────────────────────────────────────────────
    # 5. SENSOR FUSION – Tầng 2: Semantic Perception (YOLO + LiDAR/Sonar fusion)
    #    Input : /model_input/image_processed → YOLO → /yolo/detections
    #            /scan, /sonar_front/scan, /sonar_rear/scan (đã filtered)
    #            /camera/camera_info (ma trận K)
    #    Output: /fusion/obstacle_cloud (PointCloud2 → costmap obstacle layer)
    #            /fusion/semantic_obstacles (SemanticObstacleArray)
    #            /fusion/markers  (MarkerArray → RViz)
    #            /fusion/annotated_video (Image với khoảng cách)
    #    Delay 2.0s: chờ TF tree ổn định (bringup + EKF đã publish TF) trước
    #    khi semantic_fusion_node gọi tf_buffer.lookup_transform().
    # ─────────────────────────────────────────────────────────────────────────
    yolo_detector_node = Node(
        package='ackermann_rover_bringup',
        executable='yolo_detector_node.py',
        name='yolo_detector_node',
        output='screen',
        parameters=[fusion_params_file, {'use_sim_time': True}]
    )

    semantic_fusion_node = Node(
        package='ackermann_rover_bringup',
        executable='semantic_fusion_node.py',
        name='semantic_fusion_node',
        output='screen',
        parameters=[fusion_params_file, {'use_sim_time': True}]
    )

    delayed_semantic_fusion = TimerAction(
        period=2.0,
        actions=[
            GroupAction(actions=[yolo_detector_node, semantic_fusion_node])
        ]
    )

    # ─────────────────────────────────────────────────────────────────────────
    # 6. Nav2 (Planner + Controller + Costmap)
    #    Delay 5.0s: đảm bảo /scan filtered đã sẵn sàng, AMCL đã converge,
    #    EKF đã publish TF map→odom trước khi costmap khởi tạo.
    #    /fusion/obstacle_cloud được cấu hình làm obstacle_layer trong
    #    nav2_params.yaml (pointcloud_layer) để semantic obstacle ảnh hưởng
    #    vào costmap global và local.
    # ─────────────────────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'nav2.launch.py'))
    )

    mission_manager_node = Node(
        package='ackermann_rover_bringup',
        executable='mission_manager.py',
        name='mission_manager',
        output='screen'
    )

    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_launch, mission_manager_node])

    # ─────────────────────────────────────────────────────────────────────────
    # 7. Các node tiện ích
    # ─────────────────────────────────────────────────────────────────────────
    path_tracker_node = Node(
        package='ackermann_rover_bringup',
        executable='path_tracker.py',
        name='path_tracker',
        output='screen'
    )

    nav2_px4_bridge_node = Node(
        package='ackermann_rover_bringup',
        executable='nav2_px4_bridge.py',
        name='nav2_px4_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'rviz.launch.py'))
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Thứ tự khởi động:
    #   t=0s : bringup (bridge raw topics) + sensor_preprocessor + odom_to_tf
    #   t=0s : localization (EKF + AMCL)  ← dùng /scan đã filtered
    #   t=2s : yolo_detector + semantic_fusion  ← TF tree đã ổn định
    #   t=5s : Nav2 + mission_manager  ← costmap khởi tạo với /scan sẵn sàng
    #   t=0s : nav2_px4_bridge, path_tracker, rviz (không phụ thuộc thứ tự)
    # ─────────────────────────────────────────────────────────────────────────
    return LaunchDescription([
        map_name_arg,
        map_file_arg,
        use_semantic_fusion_arg,
        bringup_launch,
        sensor_preprocessor_node,
        odom_node,
        localization_launch,
        delayed_semantic_fusion,
        delayed_nav2,
        nav2_px4_bridge_node,
        path_tracker_node,
        rviz_launch,
    ])
