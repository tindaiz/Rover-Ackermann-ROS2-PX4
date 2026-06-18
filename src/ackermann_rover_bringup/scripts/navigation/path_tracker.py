#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration
import math
import copy


class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')
        self.marker_pub = self.create_publisher(Marker, '/traveled_path_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.points = []
        self.last_x = None
        self.last_y = None
        self.distance_threshold = 0.05  # Lưu điểm mới mỗi 5 cm

        self.get_logger().info('Path Tracker Node started. Publishing Marker to /traveled_path_marker')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Chỉ lưu khi khoảng cách đủ lớn
        if self.last_x is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < self.distance_threshold:
                return

        self.last_x = x
        self.last_y = y

        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = 0.05  # Nâng lên một chút để không bị chìm vào bản đồ
        self.points.append(pt)

        # Tạo Marker LINE_STRIP
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = msg.header.stamp
        marker.ns = 'traveled_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Kích thước đường vẽ
        marker.scale.x = 0.06  # Độ dày đường kẻ (6cm)

        # Màu vàng, alpha = 1 (không trong suốt)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Marker tồn tại vĩnh viễn (không tự biến mất)
        marker.lifetime = Duration(sec=0, nanosec=0)

        # Gán danh sách các điểm
        marker.points = list(self.points)

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
