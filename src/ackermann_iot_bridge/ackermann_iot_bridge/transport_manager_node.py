#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import queue
import threading
import importlib
import time

from ackermann_rover_msgs.msg import SystemHealth, Telemetry, RobotState, DownlinkCommand

class TransportManagerNode(Node):
    def __init__(self):
        super().__init__('transport_manager_node')
        
        # Async Priority Queue for Uplink
        self.uplink_queue = queue.PriorityQueue(maxsize=100)
        
        # Config (Normally read from parameters)
        #   self.declare_parameter('active_plugins', ['sim']) # LoRa test
        self.declare_parameter('active_plugins', ['mqtt'])  # MQTT real
        active_plugins = self.get_parameter('active_plugins').value
        
        self.plugins = []
        self._load_plugins(active_plugins)
        
        # Rate limiting state
        self.last_telemetry_time = self.get_clock().now()
        self.telemetry_interval = rclpy.time.Duration(seconds=0.2) # 5Hz max cho IoT
        
        # Subscriptions (Uplink)
        self.create_subscription(SystemHealth, '/iot/system_health', lambda msg: self._enqueue_uplink('health', msg), 10)
        self.create_subscription(Telemetry, '/iot/telemetry', self._throttled_telemetry_callback, 10)
        self.create_subscription(RobotState, '/iot/robot_state', lambda msg: self._enqueue_uplink('state', msg), 10)
        
        # Publisher (Downlink)
        self.downlink_pub = self.create_publisher(DownlinkCommand, '/iot/downlink_command', 10)
        
        # Start Worker Thread
        self.running = True
        self.worker_thread = self._start_worker_thread()
        
        # Watchdog Timer (Checks thread health every 5s)
        self.create_timer(5.0, self._watchdog_check)
        
        self.get_logger().info(f"Transport Manager started with plugins: {active_plugins}")

    def _load_plugins(self, plugin_names):
        for name in plugin_names:
            try:
                # E.g., 'mqtt' -> 'ackermann_iot_bridge.plugins.mqtt_transport'
                module_name = f"ackermann_iot_bridge.plugins.{name}_transport"
                module = importlib.import_module(module_name)
                # Convention: class name is TransportPlugin
                plugin_class = getattr(module, "TransportPlugin")
                
                # Mock config
                config = {}
                plugin_instance = plugin_class(config)
                plugin_instance.set_downlink_callback(self._on_downlink_received)
                
                if plugin_instance.connect():
                    self.plugins.append(plugin_instance)
                    self.get_logger().info(f"Successfully loaded plugin {name}")
                else:
                    self.get_logger().error(f"Failed to connect plugin {name}")
            except Exception as e:
                self.get_logger().error(f"Failed to load plugin {name}: {e}")

    def _throttled_telemetry_callback(self, msg):
        now = self.get_clock().now()
        if now - self.last_telemetry_time >= self.telemetry_interval:
            self.last_telemetry_time = now
            self._enqueue_uplink('telemetry', msg)

    def _enqueue_uplink(self, msg_type, msg):
        priority_map = {'health': 1, 'state': 2, 'telemetry': 3}
        prio = priority_map.get(msg_type, 3)
        
        qsize = self.uplink_queue.qsize()
        
        # Bounded Priority Policy: Reserve slots for critical messages
        if msg_type == 'telemetry' and qsize >= 80:
            return # Drop telemetry to reserve space for health/state
        if msg_type == 'state' and qsize >= 95:
            return # Drop state to reserve space for health
            
        item = (prio, time.time(), msg_type, msg, 0) # Thêm số lần retry ban đầu là 0
        try:
            self.uplink_queue.put_nowait(item)
        except queue.Full:
            pass # Discard newest item if queue is absolutely full (reservation policy already guarantees slots)
                
    def _start_worker_thread(self):
        thread = threading.Thread(target=self._worker_loop, daemon=True)
        thread.start()
        return thread

    def _watchdog_check(self):
        if not self.worker_thread.is_alive() and self.running:
            self.get_logger().error("Worker thread died! Restarting...")
            self.worker_thread = self._start_worker_thread()

    def _worker_loop(self):
        while self.running:
            try:
                # Block for a short time to allow checking self.running
                prio, timestamp, msg_type, msg, retry_count = self.uplink_queue.get(timeout=0.5)
                
                send_success = True
                for plugin in self.plugins:
                    try:
                        if msg_type == 'health':
                            plugin.send_health(msg)
                        elif msg_type == 'telemetry':
                            plugin.send_telemetry(msg)
                        elif msg_type == 'state':
                            plugin.send_robot_state(msg)
                    except Exception as e:
                        self.get_logger().error(f"Plugin error during send: {e}")
                        send_success = False
                
                if not send_success:
                    retry_count += 1
                    age_seconds = time.time() - timestamp
                    
                    # CƠ CHẾ DROP EVENTUALLY: Giới hạn 10 lần retry hoặc tồn tại quá 60 giây
                    if retry_count > 10 or age_seconds > 60.0:
                        self.get_logger().warn(f"Dropped {msg_type} message due to TTL/Retries limit (Age: {age_seconds:.1f}s, Retries: {retry_count})")
                    else:
                        # RE-ENQUEUE: Trả lại gói tin vào hàng đợi
                        item = (prio, timestamp, msg_type, msg, retry_count)
                        try:
                            self.uplink_queue.put_nowait(item)
                        except queue.Full:
                            pass
                        # BACKOFF: Ngủ 1 giây để tránh busy-loop
                        time.sleep(1.0)
                
                self.uplink_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Worker thread error: {e}")

    def _on_downlink_received(self, cmd_dict):
        """Callback for plugins to pass received downlink commands"""
        msg = DownlinkCommand()
        msg.timestamp = self.get_clock().now().to_msg()
        # Ensure dict contains required fields safely
        msg.target_node = cmd_dict.get('target_node', 0)
        msg.command_type = cmd_dict.get('command_type', 0)
        msg.sequence_id = cmd_dict.get('sequence_id', 0)
        msg.param_floats = cmd_dict.get('param_floats', [])
        msg.param_ints = cmd_dict.get('param_ints', [])
        msg.require_ack = cmd_dict.get('require_ack', False)
        msg.ttl_ms = cmd_dict.get('ttl_ms', 5000)
        
        self.downlink_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)
        for plugin in self.plugins:
            plugin.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TransportManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
