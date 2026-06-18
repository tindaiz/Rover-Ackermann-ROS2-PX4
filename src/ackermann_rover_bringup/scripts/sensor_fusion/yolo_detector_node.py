#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('rate_limit_hz', 10.0) # Rate limiting YOLO inference on CPU
        
        model_path = self.get_parameter('model_path').value
        image_topic = self.get_parameter('image_topic').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.rate_limit_hz = self.get_parameter('rate_limit_hz').value
        
        self.bridge = CvBridge()
        
        # Load YOLO model on CPU
        self.get_logger().info(f"Loading YOLO model {model_path} on CPU...")
        self.model = YOLO(model_path)
        
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        
        self.detection_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.annotated_pub = self.create_publisher(CompressedImage, '/yolo/annotated_image/compressed', 10)
        
        self.last_process_time = 0.0
        self.get_logger().info("YoloDetectorNode initialized.")

    def image_callback(self, msg: Image):
        current_time = time.time()
        # Rate limiting logic
        if (current_time - self.last_process_time) < (1.0 / self.rate_limit_hz):
            return
        
        self.last_process_time = current_time
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run inference on CPU
            results = self.model(cv_image, device='cpu', conf=self.conf_thresh, verbose=False)
            
            det_array = Detection2DArray()
            det_array.header = msg.header # Giữ nguyên timestamp của ảnh gốc
            
            if len(results) > 0:
                result = results[0]
                
                # Annotate image
                annotated_frame = result.plot()
                annotated_msg = self.bridge.cv2_to_compressed_imgmsg(annotated_frame)
                annotated_msg.header = msg.header
                self.annotated_pub.publish(annotated_msg)
                
                # Populate detections
                for box in result.boxes:
                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # Bounding box
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    w = x2 - x1
                    h = y2 - y1
                    
                    detection.bbox.center.position.x = cx
                    detection.bbox.center.position.y = cy
                    detection.bbox.size_x = w
                    detection.bbox.size_y = h
                    
                    # Hypothesis (Class ID and Confidence)
                    hyp = ObjectHypothesisWithPose()
                    class_id = int(box.cls[0].item())
                    # Gán class_name vào class_id để các node sau dễ dàng lọc (Semantic Policy)
                    class_name = self.model.names[class_id]
                    hyp.hypothesis.class_id = class_name
                    hyp.hypothesis.score = float(box.conf[0].item())
                    detection.results.append(hyp)
                    
                    det_array.detections.append(detection)
                    
            self.detection_pub.publish(det_array)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
