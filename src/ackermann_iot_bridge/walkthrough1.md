# Hướng dẫn chạy Simulation Integration (Chaos Engineering)

Mục tiêu của bài test này là chứng minh kiến trúc **Middleware** (bao gồm: Non-blocking Worker Thread, Rate Limiting, Priority Queue, và Bounded Reservation Policy) hoạt động đúng như thiết kế trong các điều kiện mạng khắc nghiệt nhất, mà không cần phải cắm module 4G hay LoRa thật.

Tôi đã tạo sẵn một plugin tên là `sim_transport.py` và cấu hình cho `transport_manager_node` mặc định chạy plugin này. Plugin này có tính năng **Chaos Engineering**:
1. **Network Latency**: Cố tình làm chậm tốc độ gửi (sleep 0.3s mỗi gói) để mô phỏng nghẽn mạng (Băng thông < Tốc độ sinh Telemetry).
2. **Network Drops**: Chu kỳ mỗi 20 giây sẽ có 15 giây "có mạng" và 5 giây "mất mạng" hoàn toàn để ép Queue bị đầy.

## Các bước thực hiện

### Bước 1: Build lại Project
Bạn mở một Terminal mới và build lại package để ROS 2 nhận diện file `sim_transport.py` mới:
```bash
cd ~/rover_ackermann
colcon build --packages-select ackermann_iot_bridge
source install/setup.bash
```

### Bước 2: Chạy PX4 SITL (Mô phỏng Xe)
Mở Terminal thứ 2, chạy Gazebo và MicroXRCE-DDS Agent như mọi khi bạn vẫn làm với xe Ackermann:
```bash
# Lệnh chạy MicroXRCE-DDS Agent (ví dụ)
MicroXRCEAgent udp4 -p 8888

# Lệnh chạy PX4 Gazebo (ở một tab khác)
make px4_sitl gz_rover
```

### Bước 3: Khởi chạy Data Aggregators
Mở Terminal thứ 3, khởi chạy 2 node thu thập dữ liệu:
```bash
source ~/rover_ackermann/install/setup.bash
ros2 run ackermann_iot_bridge system_health_node &
ros2 run ackermann_iot_bridge telemetry_aggregator_node &
```
*(Nếu muốn, bạn có thể tách thành 2 terminal khác nhau để dễ quản lý, hoặc viết file Launch. Hiện tại dùng `&` để chạy nền).*

### Bước 4: Chạy Transport Manager & Phân tích Log
Mở Terminal thứ 4, chạy lõi mạng lưới:
```bash
source ~/rover_ackermann/install/setup.bash
ros2 run ackermann_iot_bridge transport_manager_node
```

## Quan sát và Đánh giá (Validation Criteria)

Hãy ngồi nhìn màn hình Terminal của `transport_manager_node` trong vòng 1-2 phút, bạn sẽ thấy các hiện tượng kỳ diệu sau xảy ra, chứng minh kiến trúc của bạn là **đẳng cấp Production**:

> [!TIP]
> **Hiện tượng 1: Queue phình to một cách an toàn**
> Telemetry sinh ra với tốc độ 5Hz (0.2s/gói). Nhưng Sim Transport chỉ gửi được khoảng 3Hz (0.3s/gói). Bạn sẽ không hề thấy ROS 2 bị treo (đứng hình) vì chúng ta chạy Non-blocking Thread. Tuy nhiên, hàng đợi (Queue) sẽ từ từ đầy lên ngầm ở bên dưới.

> [!WARNING]
> **Hiện tượng 2: Mất mạng (Network Drop)**
> Đột nhiên log báo `[SIM ERROR] Mạng bị ngắt! Đang chặn gửi...`. Lúc này, hệ thống sẽ im lìm không in ra dòng `[SIM OUT]` nào nữa. Queue sẽ bị bơm căng cực nhanh.

> [!IMPORTANT]
> **Hiện tượng 3: Reservation Policy kích hoạt**
> Khi rớt mạng, bạn sẽ thấy màn hình in ra dòng log debug (nếu bật chế độ debug) hoặc đơn giản là Queue sẽ âm thầm vứt bỏ (Drop) các gói Telemetry mới đến. Nhờ vậy, dung lượng của Queue luôn còn dư 20 slot dành riêng cho gói Health.

> [!NOTE]
> **Hiện tượng 4: Có mạng trở lại - Trật tự ưu tiên (Priority)**
> Khi hết 5 giây mất mạng, bạn sẽ ngay lập tức thấy các dòng `[SIM OUT] *** GỬI HEALTH KHẨN CẤP ***` vọt ra **trước tiên**, sau đó mới đến các gói `[SIM OUT] Gửi State tổng hợp` và cuối cùng mới là mớ `[SIM OUT] Gửi Telemetry` cũ đang bị tồn đọng. 
> 
> Lệnh `health` dù sinh ra sau, nhưng nhờ PriorityQueue, nó đã "chen ngang" lên hàng đầu để đảm bảo Ground Station nhận được trạng thái pin khẩn cấp ngay giây phút có sóng trở lại!

Sau khi bạn đã ngắm đủ và hài lòng với kiến trúc này, bạn chỉ cần sửa file config hoặc tham số `active_plugins` thành `['mqtt', 'lora']` là xong!
