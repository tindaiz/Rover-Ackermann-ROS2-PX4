# Rover Ackermann ROS2-PX4

Dự án này cấu hình và tích hợp một xe tự hành Ackermann (Ackermann Rover) sử dụng hệ sinh thái ROS 2, PX4 Autopilot (Gazebo SITL), và Nav2 để thực hiện các nhiệm vụ điều hướng, lập bản đồ và mô phỏng.

## Cấu trúc thư mục

Workspace chứa các package ROS 2 chính sau (không bao gồm `rover_dashboard` và `m-explore-ros2`):

- **Micro-XRCE-DDS-Agent**: Dùng để làm cầu nối giao tiếp DDS giữa PX4 và ROS 2.
- **ackermann_iot_bridge**: Đảm nhiệm việc giao tiếp với các hệ thống IoT.
- **ackermann_nav2_behaviors**: Chứa các behavior (hành vi) và plugin tùy chỉnh cho Nav2 như quay đầu xe (Ackermann turnaround), lùi xe (backup), v.v.
- **ackermann_rover_bringup**: Chứa các launch file chính để khởi động toàn bộ hệ thống (robot state publisher, SLAM, Nav2, EKF, v.v.).
- **ackermann_rover_msgs**: Các định nghĩa thông điệp (messages) ROS 2 tùy chỉnh riêng cho xe Ackermann.
- **px4-ros2-interface-lib**: Thư viện hỗ trợ chuyển đổi dữ liệu và tương tác giữa PX4 và ROS 2.
- **px4_msgs**: Định nghĩa thông điệp uORB của PX4 biên dịch sang ROS 2.

## Yêu cầu hệ thống (Prerequisites)

- **OS**: Ubuntu (tương thích với phiên bản ROS 2 đang dùng, VD: Humble/Iron).
- **ROS 2**: Đã cài đặt đầy đủ (Desktop version).
- **PX4 Autopilot**: Đã được clone và cấu hình (đặt tại `~/PX4-Autopilot`).
- **Micro-XRCE-DDS-Agent**: Đã được cài đặt và build.

## Hướng dẫn sử dụng

### 1. Build không gian làm việc (Workspace)

Mở terminal, di chuyển vào thư mục workspace và tiến hành build bằng `colcon`:

```bash
cd ~/rover_ackermann
colcon build
source install/setup.bash
```

### 2. Khởi chạy dự án

Dự án có sẵn các script tự động mở nhiều tab terminal để khởi chạy PX4 SITL (Gazebo), Micro-XRCE-DDS-Agent, và ROS 2 Bringup. 

- **Chế độ Lập bản đồ và Điều hướng (Mapping & Nav):**
  Chạy script `run_all.sh` với tên map (mặc định là `nav_world`).
  ```bash
  ./run_all.sh nav_world
  ```
  Quá trình này sẽ:
  1. Mở PX4 SITL Gazebo với xe Ackermann.
  2. Bật MicroXRCEAgent (udp4 cổng 8888).
  3. Chạy ROS 2 launch file (`mapping_and_nav.launch.py`).

- **Chế độ Điều hướng độc lập (Pure Navigation):**
  Chạy script `run_all_map_behaviors.sh` để bắt đầu với tọa độ và cấu hình riêng phục vụ cho điều hướng.
  ```bash
  ./run_all_map_behaviors.sh nav_world
  ```
  Script này sẽ spawn robot ở vị trí cụ thể (VD: x=5.5, y=10.0) và chạy `pure_navigation.launch.py`.

## Tương tác và Điều khiển

- Hệ thống sử dụng Nav2 cho việc lập kế hoạch đường đi và các hành vi phục hồi (recovery behaviors).
- Sử dụng **RViz2** để thiết lập mục tiêu (2D Goal Pose) và quan sát cảm biến (LiDAR, TF tree, v.v.).
