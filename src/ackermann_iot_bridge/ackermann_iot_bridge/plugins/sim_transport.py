import time
import random
from .base_transport import BaseTransport

class TransportPlugin(BaseTransport):
    def __init__(self, config: dict):
        super().__init__(config)
        self.connected = False
        self.start_time = 0.0

    def connect(self) -> bool:
        self.connected = True
        self.start_time = time.time()
        print("[SIM] Simulation Transport Connected!")
        return True

    def disconnect(self):
        self.connected = False
        print("[SIM] Simulation Transport Disconnected!")

    def _simulate_network_conditions(self, msg_type: str):
        # 1. Chaos Engineering: Đứt mạng định kỳ
        # Chạy 15s có mạng, 5s mất mạng
        elapsed = int(time.time() - self.start_time)
        cycle = elapsed % 20
        network_up = cycle < 15
        
        if not network_up:
            print(f"[{elapsed}s] [SIM ERROR] Mạng bị ngắt! Đang chặn gửi {msg_type}...")
            # Ném exception để giả lập lỗi, Worker Thread sẽ bắt lỗi này và giữ data lại
            raise Exception("Simulated Network Down!")
            
        # 2. Giả lập độ trễ (Latency)
        # Băng thông giả lập xử lý được tối đa ~3 tin/giây (sleep 0.3s)
        # Trong khi Telemetry đẩy vào 5Hz (0.2s), Queue sẽ từ từ phình to!
        time.sleep(0.3) 

    def send_telemetry(self, msg):
        self._simulate_network_conditions("Telemetry")
        print(f"[SIM OUT] Gửi Telemetry thành công. X: {msg.position_x:.2f}, Y: {msg.position_y:.2f}")

    def send_health(self, msg):
        self._simulate_network_conditions("Health")
        print(f"[SIM OUT] *** GỬI HEALTH KHẨN CẤP *** Pin: {msg.battery_percentage}%, Trạng thái: {msg.status}")

    def send_robot_state(self, msg):
        self._simulate_network_conditions("RobotState")
        print(f"[SIM OUT] Gửi State tổng hợp. Pin: {msg.battery_percentage}%, X: {msg.position_x:.2f}")

    def send_ack(self, sequence_id: int, success: bool, message: str = ""):
        self._simulate_network_conditions("ACK")
        print(f"[SIM OUT] Gửi ACK cho lệnh {sequence_id}, Success: {success}")
