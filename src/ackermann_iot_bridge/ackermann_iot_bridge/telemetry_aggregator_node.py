#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_rover_msgs.msg import Telemetry
import math

class TelemetryAggregatorNode(Node):
    def __init__(self):
        super().__init__('telemetry_aggregator_node')
        
        self.publisher_ = self.create_publisher(Telemetry, '/iot/telemetry', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg: Odometry):
        tel_msg = Telemetry()
        # std_msgs/Header requires frame_id and timestamp
        tel_msg.header.stamp = self.get_clock().now().to_msg()
        tel_msg.header.frame_id = msg.header.frame_id
        
        tel_msg.position_x = msg.pose.pose.position.x
        tel_msg.position_y = msg.pose.pose.position.y
        tel_msg.position_z = msg.pose.pose.position.z
        
        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        tel_msg.heading_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        tel_msg.linear_speed = math.hypot(vx, vy)
        tel_msg.angular_speed = msg.twist.twist.angular.z
        
        self.publisher_.publish(tel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryAggregatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
