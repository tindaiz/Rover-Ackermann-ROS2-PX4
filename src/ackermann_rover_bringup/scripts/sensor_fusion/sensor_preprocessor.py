#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorPreprocessorNode(Node):
    def __init__(self):
        super().__init__('sensor_preprocessor_node')
        self.bridge = CvBridge()
        
        # 1. Đăng ký nhận dữ liệu độc lập
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        # Nhận dữ liệu thô từ Bridge
        self.lidar_sub = self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, 10)
        self.sonar_front_sub = self.create_subscription(LaserScan, '/sonar_front/scan_raw', self.sonar_front_callback, 10)
        self.sonar_rear_sub = self.create_subscription(LaserScan, '/sonar_rear/scan_raw', self.sonar_rear_callback, 10)
        
        # 2. Tạo Publisher cho dữ liệu đầu ra
        self.image_pub = self.create_publisher(Image, '/model_input/image_processed', 10)
        # Phát lại dữ liệu đã lọc vào /scan để Nav2 và SLAM sử dụng
        self.lidar_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.sonar_front_pub = self.create_publisher(LaserScan, '/sonar_front/scan', 10)
        self.sonar_rear_pub = self.create_publisher(LaserScan, '/sonar_rear/scan', 10)
        
        self.get_logger().info("Sensor Preprocessor Node (Independent Callbacks) has started.")

    def image_callback(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            
            h, w = cv_image.shape[:2]
            target_size = 640

            scale = min(target_size / h, target_size / w)
            new_h, new_w = int(round(h * scale)), int(round(w * scale))
            
            if (new_h, new_w) != (h, w):
                resized_img = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
            else:
                resized_img = cv_image
                
            top = (target_size - new_h) // 2
            bottom = target_size - new_h - top
            left = (target_size - new_w) // 2
            right = target_size - new_w - left

            padded_img = cv2.copyMakeBorder(
                resized_img, top, bottom, left, right, 
                cv2.BORDER_CONSTANT, value=(114, 114, 114)
            )

            rgb_image = cv2.cvtColor(padded_img, cv2.COLOR_BGR2RGB)
            rgb_image = np.ascontiguousarray(rgb_image)
            
            processed_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding='rgb8')
            processed_msg.header = image_msg.header
            
            self.image_pub.publish(processed_msg)

        except Exception as e:
            self.get_logger().error(f"YOLO preprocess error: {e}")

    def process_and_publish_scan(self, scan_msg, publisher, sensor_name="Scan", window_size=5, frame_id=None):
        try:
            processed_msg = LaserScan()
            processed_msg.header = scan_msg.header
            if frame_id:
                processed_msg.header.frame_id = frame_id
            processed_msg.angle_min = scan_msg.angle_min
            processed_msg.angle_max = scan_msg.angle_max
            processed_msg.angle_increment = scan_msg.angle_increment
            processed_msg.time_increment = scan_msg.time_increment
            processed_msg.scan_time = scan_msg.scan_time
            processed_msg.range_min = scan_msg.range_min
            processed_msg.range_max = scan_msg.range_max
            
            # Chuyển sang NumPy array để xử lý cho nhanh
            raw_ranges = np.array(scan_msg.ranges)
            
            # 1. Tiền xử lý các điểm vô cực (NaN, Inf) hoặc ngoài tầm đo
            raw_ranges[np.isnan(raw_ranges)] = 0.0
            # KHÔNG convert inf sang range_max: inf → range_max = 4.0m < obstacle_range = 4.5m
            # → costmap sẽ đánh dấu FALSE obstacle ở 4m!
            # Để inf nguyên, costmap với inf_is_valid: True sẽ dọn sạch ray đó mà không tạo obstacle.
            raw_ranges[raw_ranges < scan_msg.range_min] = 0.0
            too_large = (raw_ranges > scan_msg.range_max) & ~np.isinf(raw_ranges)
            raw_ranges[too_large] = scan_msg.range_max
            
            # 2. Bộ lọc Trung vị (Median Filter) để loại bỏ nhiễu (outliers)
            if window_size > 1:
                pad_size = window_size // 2
                # Padding 2 đầu mảng để giữ nguyên kích thước
                padded = np.pad(raw_ranges, pad_size, mode='edge')
                
                # Trích xuất các cửa sổ trượt và tính trung vị
                windows = np.array([padded[i:i+len(raw_ranges)] for i in range(window_size)])
                filtered_ranges = np.median(windows, axis=0)
            else:
                filtered_ranges = raw_ranges
            
            processed_msg.ranges = filtered_ranges.tolist()
            processed_msg.intensities = scan_msg.intensities
            
            publisher.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f"Lỗi xử lý {sensor_name}: {e}")

    def lidar_callback(self, lidar_msg):
        self.process_and_publish_scan(lidar_msg, self.lidar_pub, "LiDAR", window_size=5)
        
    def sonar_front_callback(self, sonar_msg):
        self.process_and_publish_scan(
            sonar_msg,
            self.sonar_front_pub,
            "Sonar Front",
            window_size=1,
            frame_id="sonar_front_link"
        )
        
    def sonar_rear_callback(self, sonar_msg):
        self.process_and_publish_scan(
            sonar_msg,
            self.sonar_rear_pub,
            "Sonar Rear",
            window_size=1,
            frame_id="sonar_rear_link"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SensorPreprocessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
