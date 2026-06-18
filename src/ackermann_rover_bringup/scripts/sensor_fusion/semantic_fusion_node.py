#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan, CameraInfo, PointCloud2, PointField, Image, CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from ackermann_rover_msgs.msg import SemanticObstacle, SemanticObstacleArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import collections
import time
import struct
import math
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class SemanticFusionNode(Node):
    def __init__(self):
        super().__init__('semantic_fusion_node')
        
        # Parameters
        self.declare_parameter('semantic_policy', ['person', 'car', 'bicycle', 'bus', 'truck', 'motorcycle'])
        self.declare_parameter('persistence_timeout', 1.0)
        self.declare_parameter('camera_height_offset', 0.1) # LiDAR relative to Camera Y-axis (down is positive)
        self.declare_parameter('distance_match_threshold', 1.0)
        
        self.policy = self.get_parameter('semantic_policy').value
        self.timeout = self.get_parameter('persistence_timeout').value
        self.cam_h_offset = self.get_parameter('camera_height_offset').value
        self.match_thresh = self.get_parameter('distance_match_threshold').value
        
        self.declare_parameter('inflation_classes', ['person', 'car', 'truck', 'bus', 'bicycle', 'motorcycle'])
        self.declare_parameter('inflation_radii', [1.0, 0.8, 0.8, 0.8, 0.6, 0.6])
        self.declare_parameter('goal_proximity_threshold', 1.5)
        
        inf_classes = self.get_parameter('inflation_classes').value
        inf_radii = self.get_parameter('inflation_radii').value
        self.inflation_dict = dict(zip(inf_classes, inf_radii))
        self.goal_proximity = self.get_parameter('goal_proximity_threshold').value
        
        self.scan_buffer = collections.deque(maxlen=100)
        self.sonar_front_buffer = collections.deque(maxlen=100)
        self.sonar_rear_buffer = collections.deque(maxlen=100)
        self.image_buffer = collections.deque(maxlen=30)
        self.bridge = CvBridge()
        
        self.k_matrix = None
        
        # Subscriptions
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.cam_info_cb, 1)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.sonar_front_sub = self.create_subscription(LaserScan, '/sonar_front/scan', self.sonar_front_cb, 10)
        self.sonar_rear_sub = self.create_subscription(LaserScan, '/sonar_rear/scan', self.sonar_rear_cb, 10)
        self.det_sub = self.create_subscription(Detection2DArray, '/yolo/detections', self.det_cb, 10)
        self.img_sub = self.create_subscription(CompressedImage, '/yolo/annotated_image/compressed', self.image_cb, 10)
        
        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, '/fusion/obstacle_cloud', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fusion/markers', 10)
        self.sem_pub = self.create_publisher(SemanticObstacleArray, '/fusion/semantic_obstacles', 10)
        self.annotated_video_pub = self.create_publisher(Image, '/fusion/annotated_video', 10)
        
        # Timer to publish persistent obstacles
        self.pub_timer = self.create_timer(0.1, self.publish_persistent_obstacles)   # 10Hz: đủ cho costmap (update_frequency=10Hz), tránh quá tải CPU
        
        # Goal Subscriber & TF
        self.current_goal = None
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State
        self.active_obstacles = [] # list of dict
        self.last_scan_frame_id = "base_link"
        
        self.get_logger().info("SemanticFusionNode started.")

    def goal_cb(self, msg: PoseStamped):
        self.current_goal = msg
        self.get_logger().info("Received new goal. Goal-aware inflation evaluated.")

    def cam_info_cb(self, msg: CameraInfo):
        if self.k_matrix is None:
            self.k_matrix = {
                'fx': msg.k[0],
                'cx': msg.k[2],
                'fy': msg.k[4],
                'cy': msg.k[5],
                'width': msg.width,
                'height': msg.height
            }
            self.get_logger().info("Camera matrix received.")

    def scan_cb(self, msg: LaserScan):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.scan_buffer.append((t, msg))
        
    def sonar_front_cb(self, msg: LaserScan):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.sonar_front_buffer.append((t, msg))
        
    def sonar_rear_cb(self, msg: LaserScan):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.sonar_rear_buffer.append((t, msg))

    def image_cb(self, msg: CompressedImage):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.image_buffer.append((t, msg))

    def get_nearest_msg(self, buffer, target_t, max_diff=0.5):
        nearest_msg = None
        min_diff = float('inf')
        for t, msg in buffer:
            diff = abs(t - target_t)
            if diff < min_diff:
                min_diff = diff
                nearest_msg = msg
        if min_diff <= max_diff:
            return nearest_msg
        return None

    def apply_transform(self, xs, ys, zs, trans):
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        rx = trans.transform.rotation.x
        ry = trans.transform.rotation.y
        rz = trans.transform.rotation.z
        rw = trans.transform.rotation.w
        
        R00 = 1.0 - 2.0*(ry**2 + rz**2)
        R01 = 2.0*(rx*ry - rw*rz)
        R02 = 2.0*(rx*rz + rw*ry)
        
        R10 = 2.0*(rx*ry + rw*rz)
        R11 = 1.0 - 2.0*(rx**2 + rz**2)
        R12 = 2.0*(ry*rz - rw*rx)
        
        R20 = 2.0*(rx*rz - rw*ry)
        R21 = 2.0*(ry*rz + rw*rx)
        R22 = 1.0 - 2.0*(rx**2 + ry**2)
        
        xn = R00*xs + R01*ys + R02*zs + tx
        yn = R10*xs + R11*ys + R12*zs + ty
        zn = R20*xs + R21*ys + R22*zs + tz
        return xn, yn, zn

    def project_to_camera(self, scan_msg):
        if scan_msg is None:
            return np.array([]), np.array([]), np.array([]), np.array([]), np.array([])
            
        ranges = np.array(scan_msg.ranges)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
        
        # Chỉ lấy các điểm thực sự va chạm, bỏ qua inf và max_range
        valid_idx = (ranges > scan_msg.range_min) & (ranges < scan_msg.range_max - 0.05)
        r = ranges[valid_idx]
        a = angles[valid_idx]
        
        if len(r) == 0:
            return np.array([]), np.array([]), np.array([]), np.array([]), np.array([])
        
        # Tọa độ trong hệ tọa độ cảm biến cục bộ
        x_s = r * np.cos(a)
        y_s = r * np.sin(a)
        z_s = np.zeros_like(x_s)
        
        try:
            # Lấy TF động từ sensor frame tới base_link, và từ base_link tới camera
            t_base = self.tf_buffer.lookup_transform('base_link', scan_msg.header.frame_id, rclpy.time.Time())
            t_cam = self.tf_buffer.lookup_transform('rover_ackermann_0/camera_link/front_camera', 'base_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed in semantic fusion: {e}")
            return np.array([]), np.array([]), np.array([]), np.array([]), np.array([])
            
        # Transform points
        x_b, y_b, z_b = self.apply_transform(x_s, y_s, z_s, t_base)
        x_c, y_c, z_c = self.apply_transform(x_b, y_b, z_b, t_cam)
        
        # Chuyển hệ tọa độ ROS (X tiến, Y trái, Z lên) sang Camera quang học (Z tiến, X phải, Y xuống)
        X_opt = -y_c
        Y_opt = -z_c
        Z_opt = x_c
        
        # Lọc các điểm nằm sau camera
        front_idx = Z_opt > 0.0
        X_opt = X_opt[front_idx]
        Y_opt = Y_opt[front_idx]
        Z_opt = Z_opt[front_idx]
        x_b = x_b[front_idx]
        y_b = y_b[front_idx]
        dists = np.hypot(x_b, y_b)
        
        if len(X_opt) == 0:
            return np.array([]), np.array([]), np.array([]), np.array([]), np.array([])
            
        us = (self.k_matrix['fx'] * (X_opt / Z_opt) + self.k_matrix['cx']).astype(int)
        vs = (self.k_matrix['fy'] * (Y_opt / Z_opt) + self.k_matrix['cy']).astype(int)
        
        return us, vs, x_b, y_b, dists

    def det_cb(self, msg: Detection2DArray):
        if self.k_matrix is None or (len(self.scan_buffer) == 0 and len(self.sonar_front_buffer) == 0):
            return
            
        det_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Lấy msg gần nhất của Lidar và 2 Sonar
        nearest_scan = self.get_nearest_msg(self.scan_buffer, det_t)
        nearest_sl = self.get_nearest_msg(self.sonar_front_buffer, det_t)
        nearest_sr = self.get_nearest_msg(self.sonar_rear_buffer, det_t)
        
        if nearest_scan is None and nearest_sl is None and nearest_sr is None:
            return
            
        current_time = time.time()
        new_obstacles = []
        
        # Project tất cả các cảm biến lên không gian 2D pixel sử dụng TF động
        us_l, vs_l, xb_l, yb_l, d_l = self.project_to_camera(nearest_scan)
        us_sl, vs_sl, xb_sl, yb_sl, d_sl = self.project_to_camera(nearest_sl)
        us_sr, vs_sr, xb_sr, yb_sr, d_sr = self.project_to_camera(nearest_sr)
        
        # Lọc bỏ các mảng trống trước khi gộp để tránh lỗi ép kiểu (float64 của mảng rỗng vs int64)
        valid_us = [u for u in [us_l, us_sl, us_sr] if len(u) > 0]
        if not valid_us:
            return
            
        us = np.concatenate(valid_us)
        vs = np.concatenate([v for v in [vs_l, vs_sl, vs_sr] if len(v) > 0])
        xs = np.concatenate([x for x in [xb_l, xb_sl, xb_sr] if len(x) > 0])
        ys = np.concatenate([y for y in [yb_l, yb_sl, yb_sr] if len(y) > 0])
        ranges = np.concatenate([d for d in [d_l, d_sl, d_sr] if len(d) > 0])
        
        for det in msg.detections:
            if not det.results:
                continue
                
            class_name = det.results[0].hypothesis.class_id
            conf = det.results[0].hypothesis.score
            
            if class_name not in self.policy:
                continue
                
            bx = det.bbox.center.position.x
            by = det.bbox.center.position.y
            bw = det.bbox.size_x
            bh = det.bbox.size_y
            
            x_min = bx - bw/2
            x_max = bx + bw/2
            y_min = by - bh/2
            y_max = by + bh/2
            
            inside_idx = (us >= x_min) & (us <= x_max) & (vs >= y_min) & (vs <= y_max)
            pts_x = xs[inside_idx]
            pts_y = ys[inside_idx]
            pts_r = ranges[inside_idx]
            
            if len(pts_r) > 0:
                min_idx = np.argmin(pts_r)
                obj_dist = pts_r[min_idx]
                obj_x = pts_x[min_idx]
                obj_y = pts_y[min_idx]
                
                new_obstacles.append({
                    'class': class_name,
                    'conf': conf,
                    'x': obj_x,
                    'y': obj_y,
                    'z': 0.0,
                    'dist': obj_dist,
                    'time': current_time,
                    'points': np.column_stack((pts_x, pts_y, np.zeros_like(pts_x))),
                    'bbox_x': int(bx),
                    'bbox_y': int(y_min)
                })
                
        # Merge new_obstacles into active_obstacles
        for new_obs in new_obstacles:
            matched = False
            for act_obs in self.active_obstacles:
                if act_obs['class'] == new_obs['class']:
                    dist_diff = math.hypot(act_obs['x'] - new_obs['x'], act_obs['y'] - new_obs['y'])
                    if dist_diff < self.match_thresh:
                        act_obs['x'] = new_obs['x']
                        act_obs['y'] = new_obs['y']
                        act_obs['dist'] = new_obs['dist']
                        act_obs['time'] = new_obs['time']
                        act_obs['points'] = new_obs['points']
                        act_obs['conf'] = max(act_obs['conf'], new_obs['conf'])
                        matched = True
                        break
            if not matched:
                self.active_obstacles.append(new_obs)

        # Draw distances on the matching image and publish
        matched_img_msg = None
        min_img_diff = float('inf')
        for t, img_msg in self.image_buffer:
            diff = abs(t - det_t)
            if diff < min_img_diff:
                min_img_diff = diff
                matched_img_msg = img_msg
                
        if matched_img_msg is not None and min_img_diff < 0.2:
            try:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(matched_img_msg, desired_encoding='bgr8')
                for new_obs in new_obstacles:
                    text = f"Dist: {new_obs['dist']:.2f}m"
                    cv2.putText(cv_image, text, (new_obs['bbox_x'] - 40, max(20, new_obs['bbox_y'] - 10)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                out_msg.header = matched_img_msg.header
                self.annotated_video_pub.publish(out_msg)
            except Exception as e:
                self.get_logger().error(f"Error drawing distance on image: {e}")

    def publish_persistent_obstacles(self):
        current_time = time.time()
        self.active_obstacles = [obs for obs in self.active_obstacles if (current_time - obs['time']) <= self.timeout]
        
        sem_array = SemanticObstacleArray()
        sem_array.header.stamp = self.get_clock().now().to_msg()
        sem_array.header.frame_id = self.last_scan_frame_id
        
        marker_array = MarkerArray()
        
        # Add DELETEALL marker to clear previous markers
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_all_marker)
        
        all_points = []
        
        for i, obs in enumerate(self.active_obstacles):
            sem_msg = SemanticObstacle()
            sem_msg.class_name = obs['class']
            sem_msg.confidence = obs['conf']
            sem_msg.position.x = float(obs['x'])
            sem_msg.position.y = float(obs['y'])
            sem_msg.position.z = 0.0
            sem_msg.distance = float(obs['dist'])
            sem_array.obstacles.append(sem_msg)
            
            # Goal-Aware Semantic Inflation
            radius = self.inflation_dict.get(obs['class'], 0.0)
            disable_inflation = False
            
            if radius > 0.0 and self.current_goal is not None:
                try:
                    # Transform goal to lidar frame
                    trans = self.tf_buffer.lookup_transform(self.last_scan_frame_id, self.current_goal.header.frame_id, rclpy.time.Time())
                    goal_in_lidar = tf2_geometry_msgs.do_transform_pose(self.current_goal.pose, trans)
                    dist_to_goal = math.hypot(goal_in_lidar.position.x - float(obs['x']), goal_in_lidar.position.y - float(obs['y']))
                    
                    if dist_to_goal < self.goal_proximity:
                        disable_inflation = True
                        
                except Exception as e:
                    pass # Ignore TF errors and keep inflation
                    
            if radius > 0.0 and not disable_inflation:
                num_vpts = max(12, int(radius * 30))
                for angle in np.linspace(0, 2*math.pi, num_vpts, endpoint=False):
                    vx = float(obs['x']) + radius * math.cos(angle)
                    vy = float(obs['y']) + radius * math.sin(angle)
                    all_points.append([vx, vy, 0.0])
            
            all_points.extend(obs['points'])
            
            # Marker A: Sphere
            sphere = Marker()
            sphere.header = sem_array.header
            sphere.ns = "semantic_sphere"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(obs['x'])
            sphere.pose.position.y = float(obs['y'])
            sphere.pose.position.z = 0.5
            sphere.scale.x = 0.5
            sphere.scale.y = 0.5
            sphere.scale.z = 0.5
            sphere.color.a = 0.6
            sphere.color.r = 1.0
            sphere.color.g = 0.0
            sphere.color.b = 0.0
            # Set lifetime slightly larger than timeout so it disappears if not updated
            sphere.lifetime.sec = int(self.timeout)
            sphere.lifetime.nanosec = int((self.timeout - int(self.timeout)) * 1e9)
            marker_array.markers.append(sphere)
            
            # Marker B: Text
            text = Marker()
            text.header = sem_array.header
            text.ns = "semantic_text"
            text.id = i + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(obs['x'])
            text.pose.position.y = float(obs['y'])
            text.pose.position.z = 1.0
            text.scale.z = 0.3
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.text = f"{obs['class']} ({obs['conf']:.2f})"
            # Set lifetime slightly larger than timeout so it disappears if not updated
            text.lifetime.sec = int(self.timeout)
            text.lifetime.nanosec = int((self.timeout - int(self.timeout)) * 1e9)
            marker_array.markers.append(text)
            
        self.sem_pub.publish(sem_array)
        self.marker_pub.publish(marker_array)
        
        if len(all_points) > 0:
            pc_msg = self.create_pointcloud2(all_points, sem_array.header)
            self.pc_pub.publish(pc_msg)
        else:
            pc_msg = self.create_pointcloud2([], sem_array.header)
            self.pc_pub.publish(pc_msg)

    def create_pointcloud2(self, points, header):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        buffer = bytearray(msg.row_step)
        for i, pt in enumerate(points):
            struct.pack_into('<fff', buffer, i * 12, pt[0], pt[1], pt[2])
            
        msg.data = buffer
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = SemanticFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
