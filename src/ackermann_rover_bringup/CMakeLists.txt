cmake_minimum_required(VERSION 3.8)
project(ackermann_rover_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY
  config
  launch
  urdf
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/localization/odom_to_tf.py
  scripts/utils/save_map.sh
  scripts/utils/save_posegraph.sh
  scripts/utils/mission_manager.py
  scripts/navigation/nav2_px4_bridge.py
  scripts/sensor_fusion/sensor_preprocessor.py
  scripts/sensor_fusion/yolo_detector_node.py
  scripts/sensor_fusion/semantic_fusion_node.py
  scripts/navigation/collision_detector_node.py
  scripts/sensor_fusion/encoder_odom_node.py
  scripts/navigation/path_tracker.py
  scripts/navigation/trajectory_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
