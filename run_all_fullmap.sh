#!/bin/bash
# Script tự động khởi động hệ thống PX4 + ROS2 + RViz2
# Hướng dẫn sử dụng: ./run_all.sh [tên_map]
# Ví dụ: ./run_all.sh nav_world hoặc ./run_all.sh map_1

# Mặc định map là nav_world nếu không truyền vào
MAP_NAME=${1:-nav_world}

echo "Bắt đầu khởi động hệ thống với map: $MAP_NAME"

# Mở 1 terminal với tab đầu tiên (PX4)
gnome-terminal \
--tab --title="PX4 SITL & Gazebo" \
-- bash -c "cd ~/PX4-Autopilot && PX4_GZ_WORLD=$MAP_NAME make px4_sitl gz_rover_ackermann; exec bash"

# Đợi PX4/Gazebo khởi động
sleep 5

# Mở tab thứ 2 trong cùng terminal
gnome-terminal \
--tab --title="Micro-XRCE-DDS-Agent" \
-- bash -c "MicroXRCEAgent udp4 -p 8888; exec bash"

# Đợi Agent kết nối
sleep 15

# Mở tab thứ 3
gnome-terminal \
--tab --title="ROS 2 Bringup" \
-- bash -c "cd ~/rover_ackermann && source install/setup.bash && ros2 launch ackermann_rover_bringup pure_navigation.launch.py map_name:=$MAP_NAME; exec bash"

echo "Đã mở 1 terminal với 3 tab theo thứ tự!"