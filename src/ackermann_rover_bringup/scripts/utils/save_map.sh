#!/bin/bash
# Script lưu bản đồ (map.pgm và map.yaml) vào thư mục maps/

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
MAP_DIR="$SCRIPT_DIR/../maps"

# Tên bản đồ mặc định nếu không truyền tham số
MAP_NAME=${1:-"my_map_2d"}

echo "Đang lưu bản đồ vào: $MAP_DIR/$MAP_NAME"
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$MAP_NAME" 
echo "Đã lưu xong!"
