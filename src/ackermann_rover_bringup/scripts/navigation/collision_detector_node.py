#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

class CollisionDetectorNode(Node):
    def __init__(self):
        super().__init__('collision_detector_node')
        
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sonar_f_sub = self.create_subscription(LaserScan, '/sonar_front/scan', self.sonar_f_cb, 10)
        self.sonar_r_sub = self.create_subscription(LaserScan, '/sonar_rear/scan', self.sonar_r_cb, 10)
        
        self.current_cmd_v = 0.0
        self.current_odom_v = 0.0
        self.front_dist = float('inf')
        self.rear_dist = float('inf')
        
        self.stuck_start_time = None
        self.stuck_duration_threshold = 1.0  # seconds
        
        self.timer = self.create_timer(0.1, self.check_collision)
        self.get_logger().info("Collision Detector Node started.")

    def cmd_cb(self, msg):
        self.current_cmd_v = msg.linear.x
        
    def odom_cb(self, msg):
        # We take the absolute value or direct value. Let's keep direct.
        self.current_odom_v = msg.twist.twist.linear.x
        
    def sonar_f_cb(self, msg):
        ranges = np.array(msg.ranges)
        # Handle inf and valid ranges
        valid_ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max) & ~np.isinf(ranges) & ~np.isnan(ranges)]
        if len(valid_ranges) > 0:
            self.front_dist = np.min(valid_ranges)
        else:
            self.front_dist = float('inf')
            
    def sonar_r_cb(self, msg):
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[(ranges > msg.range_min) & (ranges < msg.range_max) & ~np.isinf(ranges) & ~np.isnan(ranges)]
        if len(valid_ranges) > 0:
            self.rear_dist = np.min(valid_ranges)
        else:
            self.rear_dist = float('inf')

    def check_collision(self):
        is_stuck = False
        collision_msg = ""
        
        # Threshods
        cmd_thresh = 0.15      # Vận tốc điều khiển tối thiểu để coi là xe đang cố di chuyển
        odom_thresh = 0.05     # Vận tốc thực tế tối đa (nếu nhỏ hơn tức là xe không nhúc nhích)
        dist_thresh = 0.40     # Khoảng cách tối thiểu từ sonar đến vật cản
        
        # Xe đang cố tiến nhưng bị kẹt, và có vật cản phía trước
        if self.current_cmd_v > cmd_thresh and abs(self.current_odom_v) < odom_thresh:
            if self.front_dist < dist_thresh:
                is_stuck = True
                collision_msg = f"FRONT COLLISION DETECTED! Cmd: {self.current_cmd_v:.2f} m/s, Odom: {self.current_odom_v:.2f} m/s, Sonar Dist: {self.front_dist:.2f} m"
                
        # Xe đang cố lùi nhưng bị kẹt, và có vật cản phía sau
        elif self.current_cmd_v < -cmd_thresh and abs(self.current_odom_v) < odom_thresh:
            if self.rear_dist < dist_thresh:
                is_stuck = True
                collision_msg = f"REAR COLLISION DETECTED! Cmd: {self.current_cmd_v:.2f} m/s, Odom: {self.current_odom_v:.2f} m/s, Sonar Dist: {self.rear_dist:.2f} m"
                
        if is_stuck:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if self.stuck_start_time is None:
                self.stuck_start_time = current_time
            elif (current_time - self.stuck_start_time) > self.stuck_duration_threshold:
                self.get_logger().error(f"🚨 {collision_msg}")
                # Có thể thêm logic cancel Navigation action ở đây trong tương lai
        else:
            self.stuck_start_time = None

def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
