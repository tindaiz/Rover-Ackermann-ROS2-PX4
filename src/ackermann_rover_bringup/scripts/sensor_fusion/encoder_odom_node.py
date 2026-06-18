#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math
import time

class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_odom_node')
        
        # Cấu hình các ROS parameters
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheelbase', 0.5)
        self.declare_parameter('rear_left_joint', 'wheel_rear_left_joint')
        self.declare_parameter('rear_right_joint', 'wheel_rear_right_joint')
        self.declare_parameter('front_left_steer_joint', 'wheel_front_left_steering_joint')
        self.declare_parameter('front_right_steer_joint', 'wheel_front_right_steering_joint')
        self.declare_parameter('publish_tf', False)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.rear_left_joint = self.get_parameter('rear_left_joint').value
        self.rear_right_joint = self.get_parameter('rear_right_joint').value
        self.front_left_steer_joint = self.get_parameter('front_left_steer_joint').value
        self.front_right_steer_joint = self.get_parameter('front_right_steer_joint').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.publisher = self.create_publisher(Odometry, '/odom_encoder', 10)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.get_logger().info(f"Encoder Odom Node started with wheel_radius={self.wheel_radius}, wheelbase={self.wheelbase}")

    def joint_state_callback(self, msg):
        try:
            # Lấy index của các khớp trong message
            idx_rl = msg.name.index(self.rear_left_joint)
            idx_rr = msg.name.index(self.rear_right_joint)
            idx_fl_steer = msg.name.index(self.front_left_steer_joint)
            idx_fr_steer = msg.name.index(self.front_right_steer_joint)
        except ValueError as e:
            # Nếu chưa đủ các khớp, bỏ qua
            return

        # Vận tốc góc của bánh sau (rad/s)
        w_rl = msg.velocity[idx_rl]
        w_rr = msg.velocity[idx_rr]
        
        # Góc bẻ lái của bánh trước (rad)
        delta_left = msg.position[idx_fl_steer]
        delta_right = msg.position[idx_fr_steer]
        
        # Tính toán góc lái trung bình
        delta = (delta_left + delta_right) / 2.0
        
        # Vận tốc dài của xe (m/s)
        v_left = w_rl * self.wheel_radius
        v_right = w_rr * self.wheel_radius
        v = (v_left + v_right) / 2.0
        
        # Vận tốc góc của xe (rad/s)
        # Đối với xe Ackermann: w = v * tan(delta) / L
        w = v * math.tan(delta) / self.wheelbase
        
        # Sử dụng thời gian chính xác từ JointState thay vì tự đo
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        
        if not hasattr(self, 'last_time_sec'):
            self.last_time_sec = current_time_sec
            return
            
        dt = current_time_sec - self.last_time_sec
        self.last_time_sec = current_time_sec
        
        # Tính toán tích phân vị trí
        delta_x = (v * math.cos(self.th)) * dt
        delta_y = (v * math.sin(self.th)) * dt
        delta_th = w * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Đặt vị trí
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Chuyển yaw sang quaternion
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)
        
        # Đặt vận tốc
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        
        # Ma trận hiệp phương sai (giá trị tượng trưng)
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01
        
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[14] = 99999
        odom.twist.covariance[21] = 99999
        odom.twist.covariance[28] = 99999
        odom.twist.covariance[35] = 0.01
        
        self.publisher.publish(odom)
        
        if self.publish_tf:
            # Nếu cần publish tf odom -> base_link (EKF thường đảm nhận việc này)
            pass

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
