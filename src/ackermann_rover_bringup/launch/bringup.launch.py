import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'ackermann_rover_bringup'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Tham số tên bản đồ truyền từ ngoài vào (ví dụ: map1, nav_world)
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='nav_world',
        description='Tên của môi trường Gazebo để cấu hình bridge đúng topic'
    )

    # ros_gz_bridge Node sử dụng tham số động thay vì đọc file tĩnh
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Sử dụng perfect odom từ Gazebo mặc định. Khi chạy thực tế, comment dòng dưới đây và dùng joint_state
            '/model/rover_ackermann/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ['/world/', LaunchConfiguration('world_name'), '/model/rover_ackermann_0/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
            ['/world/', LaunchConfiguration('world_name'), '/model/rover_ackermann_0/model/lidar_2d_v2_sensor/link/link/sensor/lidar_2d_v2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            ['/world/', LaunchConfiguration('world_name'), '/model/rover_ackermann_0/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            ['/world/', LaunchConfiguration('world_name'), '/model/rover_ackermann_0/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'],
            '/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/sonar_front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/sonar_rear/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            ('/model/rover_ackermann/odometry', '/odom'),
            (['/world/', LaunchConfiguration('world_name'), '/model/rover_ackermann_0/joint_state'], '/joint_states'),
            ('/front_camera/camera_info', '/camera/camera_info'),
            (['/world/', LaunchConfiguration('world_name'), '/model/rover_ackermann_0/model/lidar_2d_v2_sensor/link/link/sensor/lidar_2d_v2/scan'], '/scan_raw'),
            (['/world/', LaunchConfiguration('world_name'), '/model/rover_ackermann_0/link/base_link/sensor/imu_sensor/imu'], '/imu'),
            (['/world/', LaunchConfiguration('world_name'), '/model/rover_ackermann_0/link/base_link/sensor/navsat_sensor/navsat'], '/gps/fix'),
            ('/sonar_front/scan', '/sonar_front/scan_raw'),
            ('/sonar_rear/scan', '/sonar_rear/scan_raw'),
        ],
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True
        }],
        output='screen'
    )

    # Image Bridge Node (Tối ưu cho việc bridge ảnh băng thông cao từ Gazebo sang ROS 2)
    image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/front_camera/image'],
        output='screen',
        remappings=[('/front_camera/image', '/camera/image_raw')],
        parameters=[{'use_sim_time': True}]
    )

    # Đường dẫn file URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover_ackermann.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # Các node chuyển tiếp frame_id từ URDF chuẩn sang frame_id thực tế mà Gazebo gán trong message header
    tf_laser_gz = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser_gz',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--roll', '0', '--pitch', '0', '--yaw', '0', 
            '--frame-id', 'laser', '--child-frame-id', 'rover_ackermann_0/lidar_2d_v2_sensor/link/lidar_2d_v2'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    tf_imu_gz = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_gz',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--roll', '0', '--pitch', '0', '--yaw', '0', 
            '--frame-id', 'imu_link', '--child-frame-id', 'rover_ackermann_0/base_link/imu_sensor'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    tf_camera_gz = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_camera_gz',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--roll', '0', '--pitch', '0', '--yaw', '0', 
            '--frame-id', 'camera_link', '--child-frame-id', 'rover_ackermann_0/camera_link/front_camera'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    tf_sonar_front_gz = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_sonar_front_gz',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--roll', '0', '--pitch', '0', '--yaw', '0', 
            '--frame-id', 'sonar_front_link', '--child-frame-id', 'rover_ackermann_0/sonar_front_link/sonar_front'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    tf_sonar_rear_gz = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_sonar_rear_gz',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--roll', '0', '--pitch', '0', '--yaw', '0', 
            '--frame-id', 'sonar_rear_link', '--child-frame-id', 'rover_ackermann_0/sonar_rear_link/sonar_rear'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    tf_gps_gz = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_gps_gz',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0', 
            '--roll', '0', '--pitch', '0', '--yaw', '0', 
            '--frame-id', 'gps_link', '--child-frame-id', 'rover_ackermann_0/base_link/navsat_sensor'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Node tính odom từ encoder. Khi cần chạy thực tế, bỏ comment dòng này và thêm vào LaunchDescription
    # encoder_odom_node = Node(
    #     package='ackermann_rover_bringup',
    #     executable='encoder_odom_node.py',
    #     name='encoder_odom_node',
    #     output='screen',
    #     parameters=[{
    #         'wheel_radius': 0.1,
    #         'wheelbase': 0.5,
    #         'rear_left_joint': 'wheel_rear_left_joint',
    #         'rear_right_joint': 'wheel_rear_right_joint',
    #         'front_left_steer_joint': 'wheel_front_left_steering_joint',
    #         'front_right_steer_joint': 'wheel_front_right_steering_joint',
    #         'publish_tf': False,
    #         'use_sim_time': True
    #     }]
    # )

    return LaunchDescription([
        world_name_arg,
        bridge_node,
        image_bridge_node,
        robot_state_publisher,
        tf_laser_gz,
        tf_imu_gz,
        tf_camera_gz,
        tf_sonar_front_gz,
        tf_sonar_rear_gz,
        tf_gps_gz
        # encoder_odom_node
    ])
