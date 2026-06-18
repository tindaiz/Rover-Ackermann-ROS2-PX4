#!/bin/bash
# Script lưu PoseGraph từ slam_toolbox dùng cho chạy Localization/Lifelong SLAM lần sau

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
MAP_DIR="$SCRIPT_DIR/../maps"

# Tên PoseGraph mặc định nếu không truyền tham số
MAP_NAME=${1:-"my_posegraph"}

# Đảm bảo đường dẫn map là tuyệt đối
MAP_PATH="$MAP_DIR/$MAP_NAME"

echo "Đang yêu cầu slam_toolbox lưu PoseGraph vào: $MAP_PATH"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$MAP_PATH'}"
echo "Lệnh đã được gửi đi! Vui lòng kiểm tra thư mục maps/ xem đã xuất hiện file .posegraph và .data chưa."
