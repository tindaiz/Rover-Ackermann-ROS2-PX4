import struct
from .base_transport import BaseTransport

class TransportPlugin(BaseTransport):
    def __init__(self, config: dict):
        super().__init__(config)
        self.connected = False
        self.magic_header = b'\xAA\xBB'
        self.seq_id = 0

    def connect(self) -> bool:
        # Dummy UART connect
        self.connected = True
        return True

    def disconnect(self):
        self.connected = False

    def _calc_checksum(self, payload: bytes) -> bytes:
        # Simple 8-bit Checksum (Not a true CRC polynomial)
        checksum = sum(payload) % 256
        return struct.pack('<B', checksum)

    def send_telemetry(self, msg):
        pass # LoRa might only send RobotState due to low bandwidth

    def send_health(self, msg):
        pass

    def send_robot_state(self, msg):
        if not self.connected: return
        # Format: uint8 seq(1), Float X(4), Float Y(4), uint8 battery(1), uint8 status(1)
        payload = struct.pack('<BffBB', self.seq_id, msg.position_x, msg.position_y, int(msg.battery_percentage), msg.system_status)
        checksum = self._calc_checksum(payload)
        packet = self.magic_header + payload + checksum
        
        # Increment and wrap seq_id (uint8)
        self.seq_id = (self.seq_id + 1) % 256
        
        # serial.write(packet)
        pass

    def send_ack(self, sequence_id: int, success: bool, message: str = ""):
        if not self.connected: return
        # Format: seq(uint8), ack_seq(uint32), success(bool/uint8)
        # Message string is usually dropped in LoRa to save bandwidth
        payload = struct.pack('<B I B', self.seq_id, sequence_id, 1 if success else 0)
        checksum = self._calc_checksum(payload)
        packet = self.magic_header + payload + checksum
        
        self.seq_id = (self.seq_id + 1) % 256
        # serial.write(packet)
        pass
