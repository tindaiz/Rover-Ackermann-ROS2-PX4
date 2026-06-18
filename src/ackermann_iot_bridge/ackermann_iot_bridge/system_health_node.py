#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import psutil

from ackermann_rover_msgs.msg import SystemHealth
from px4_msgs.msg import BatteryStatus, VehicleStatus

class SystemHealthNode(Node):
    def __init__(self):
        super().__init__('system_health_node')

        self.publisher_ = self.create_publisher(SystemHealth, '/iot/system_health', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # 1Hz

        # Subscriptions from PX4 (PX4 uses BEST_EFFORT reliability)
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status_v1',
            self.battery_callback,
            qos_profile_sensor_data
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sensor_data
        )

        # Internal state
        self.battery_percentage = 0.0
        self.battery_voltage = 0.0
        self.px4_status = 0 # Not connected
        self.hardware_id = "ACKERMANN_01"

    def battery_callback(self, msg):
        # 3S Battery: 3 * 4.2V = 12.6V (Max), 3 * 3.4V = 10.2V (Min)
        V_MAX = 12.6
        V_MIN = 10.2
        
        self.battery_voltage = msg.voltage_v
        
        # Calculate percentage based on voltage linearly
        clamped_v = max(V_MIN, min(V_MAX, self.battery_voltage))
        self.battery_percentage = ((clamped_v - V_MIN) / (V_MAX - V_MIN)) * 100.0

    def vehicle_status_callback(self, msg):
        # Maps PX4 nav_state to our status
        # Just simple mapping for now
        self.px4_status = msg.nav_state

    def timer_callback(self):
        msg = SystemHealth()
        msg.timestamp = self.get_clock().now().to_msg()
        
        # CPU/RAM
        msg.cpu_usage = float(psutil.cpu_percent())
        msg.ram_usage = float(psutil.virtual_memory().percent)
        msg.disk_usage = float(psutil.disk_usage('/').percent)
        
        # Hardware
        msg.battery_percentage = float(self.battery_percentage)
        msg.battery_voltage = float(self.battery_voltage)
        msg.hardware_id = self.hardware_id
        
        # Temp is not available by default on all systems without specific sensors
        msg.temperature = float('nan')
        
        # Status
        msg.status = SystemHealth.STATUS_OK if self.px4_status > 0 else SystemHealth.STATUS_WARNING

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
