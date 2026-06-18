import json
import time
import paho.mqtt.client as mqtt
from .base_transport import BaseTransport

class TransportPlugin(BaseTransport):
    def __init__(self, config: dict):
        super().__init__(config)
        self.connected = False
        
        try:
            from paho.mqtt.enums import CallbackAPIVersion
            self.client = mqtt.Client(CallbackAPIVersion.VERSION1)
        except (ImportError, AttributeError):
            self.client = mqtt.Client()
            
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        
        # Localhost by default
        self.broker = config.get("broker", "localhost")
        self.port = config.get("port", 1883)
        self.topic_prefix = config.get("topic_prefix", "rover")

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.connected = True
            print(f"[MQTT] Connected to {self.broker}:{self.port}")
            client.subscribe(f"{self.topic_prefix}/downlink")
        else:
            print(f"[MQTT] Connection failed with code {rc}")

    def _on_disconnect(self, client, userdata, rc):
        self.connected = False
        print(f"[MQTT] Disconnected with code {rc}")

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            if self.downlink_callback:
                self.downlink_callback(payload)
        except Exception as e:
            print(f"[MQTT] Error parsing message: {e}")

    def connect(self) -> bool:
        try:
            self.client.will_set(f"{self.topic_prefix}/health", payload=json.dumps({"status": "offline"}), qos=1, retain=True)
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            time.sleep(0.5) # Give it a moment to connect
            return True
        except Exception as e:
            print(f"[MQTT] Connection error: {e}")
            return False

    def disconnect(self):
        if self.connected:
            self.client.loop_stop()
            self.client.disconnect()
            self.connected = False

    def send_telemetry(self, msg):
        if not self.connected: return
        data = {
            "v": "1.0",
            "type": "telemetry",
            "ts": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            "pos": [msg.position_x, msg.position_y, msg.position_z],
            "yaw": msg.heading_yaw,
            "speed": msg.linear_speed
        }
        self.client.publish(f"{self.topic_prefix}/telemetry", json.dumps(data), qos=0)

    def send_health(self, msg):
        if not self.connected: return
        data = {
            "v": "1.0",
            "type": "health",
            "ts": msg.timestamp.sec + msg.timestamp.nanosec / 1e9,
            "battery": msg.battery_percentage,
            "status": msg.status
        }
        self.client.publish(f"{self.topic_prefix}/health", json.dumps(data), qos=1)

    def send_robot_state(self, msg):
        if not self.connected: return
        data = {
            "v": "1.0",
            "type": "state",
            "ts": msg.timestamp.sec + msg.timestamp.nanosec / 1e9,
            "pos": [msg.position_x, msg.position_y],
            "battery": msg.battery_percentage
        }
        self.client.publish(f"{self.topic_prefix}/state", json.dumps(data), qos=1)

    def send_ack(self, sequence_id: int, success: bool, message: str = ""):
        if not self.connected: return
        data = {
            "v": "1.0",
            "type": "ack",
            "ts": time.time(),
            "seq": sequence_id,
            "success": success,
            "msg": message
        }
        self.client.publish(f"{self.topic_prefix}/ack", json.dumps(data), qos=1)
