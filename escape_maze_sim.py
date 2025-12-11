#!/usr/bin/env python3
"""
Escape Maze Clue Detector
Detects shapes, colors, QR codes, and the key symbol using YOLOv8 and OpenCV
Integrated with ROS2 for simulation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import json

class MazeClueDetector(Node):
    def __init__(self):
        super().__init__('maze_clue_detector')
        
        # Publishers
        self.clue_pub = self.create_publisher(String, '/maze/clue_detected', 10)
        self.debug_pub = self.create_publisher(String, '/debug/clue_status', 10)
        
        # Subscriber to simulation camera
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Initialize components
        self.bridge = CvBridge()
        
        # Load YOLOv8 model (custom trained on maze objects)
        try:
            # Try custom model first, fall back to pretrained
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info("YOLOv8 model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.model = None
        
        # Target objects for escape maze
        self.target_shapes = ['triangle', 'square', 'circle', 'star']
        self.target_colors = ['red', 'green', 'yellow', 'blue']
        
        # Color ranges in HSV
        self.color_ranges = {
            'red': [(0, 100, 100), (10, 255, 255)],
            'green': [(40, 100, 100), (80, 255, 255)],
            'yellow': [(20, 100, 100), (40, 255, 255)],
            'blue': [(90, 100, 100), (130, 255, 255)]
        }
        
        # QR code detector
        self.qr_detector = cv2.QRCodeDetector()
        
        self.get_logger().info("Escape Maze Clue Detector initialized")

    def detect_shapes_yolo(self, image):
        """Detect shapes using YOLOv8"""
        if self.model is None:
            return None
        
        results = self.model(image, verbose=False)
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    label = self.model.names[cls_id]
                    confidence = float(box.conf[0])
                    
                    if label in self.target_shapes and confidence > 0.5:
                        return label
        
        return None

    def detect_shapes_contour(self, image):
        """Fallback shape detection using contour analysis"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 500:
                continue
                
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            sides = len(approx)
            
            if sides == 3:
                return 'triangle'
            elif sides == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / h
                if 0.9 <= aspect_ratio <= 1.1:
                    return 'square'
            elif sides > 8:
                return 'circle'
        
        return None

    def detect_color(self, image):
        """Detect dominant color in image"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        for color_name, (lower, upper) in self.color_ranges.items():
            lower_np = np.array(lower)
            upper_np = np.array(upper)
            mask = cv2.inRange(hsv, lower_np, upper_np)
            
            # Count colored pixels
            if np.sum(mask) > 10000:  # Adjust threshold as needed
                return color_name
        
        return None

    def detect_qr_code(self, image):
        """Detect and decode QR codes"""
        data, bbox, _ = self.qr_detector.detectAndDecode(image)
        
        if data and bbox is not None:
            self.get_logger().info(f"QR Code detected: {data}")
            return data.strip()
        
        return None

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            detected_clues = []
            
            # 1. Detect QR codes (highest priority)
            qr_data = self.detect_qr_code(cv_image)
            if qr_data:
                clue = {
                    'type': 'qr',
                    'value': qr_data,
                    'zone': self.infer_zone_from_clue(qr_data)
                }
                detected_clues.append(clue)
            
            # 2. Detect shapes
            shape = self.detect_shapes_yolo(cv_image)
            if not shape:
                shape = self.detect_shapes_contour(cv_image)
            
            if shape:
                clue = {
                    'type': 'shape',
                    'value': shape,
                    'zone': 'shape_gate' if shape != 'star' else 'final_exit'
                }
                detected_clues.append(clue)
            
            # 3. Detect colors
            color = self.detect_color(cv_image)
            if color:
                clue = {
                    'type': 'color',
                    'value': color,
                    'zone': 'color_hall'
                }
                detected_clues.append(clue)
            
            # Publish all detected clues
            for clue in detected_clues:
                clue_msg = String()
                clue_msg.data = json.dumps(clue)
                self.clue_pub.publish(clue_msg)
                self.get_logger().info(f"Clue detected: {clue}")
                
                # Also publish debug info
                debug_msg = String()
                debug_msg.data = f"Detected {clue['type']}: {clue['value']}"
                self.debug_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def infer_zone_from_clue(self, clue_value):
        """Infer which zone we're in based on clue content"""
        clue_lower = clue_value.lower()
        
        if 'zone' in clue_lower:
            return 'qr_chamber'
        elif 'key' in clue_lower or 'star' in clue_lower:
            return 'final_zone'
        elif 'proceed' in clue_lower or 'go to' in clue_lower:
            return 'navigation_instruction'
        
        return 'unknown'

def main(args=None):
    rclpy.init(args=args)
    node = MazeClueDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
