#!/usr/bin/env python3
from .yolov8_detector import YOLODetector
import torch
import numpy as np
import cv2
from time import time
from ultralytics import YOLO
import supervision as sv
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import Image
from utbots_msgs.msg import BoundingBoxes, BoundingBox
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from utbots_actions.action import YOLODetection

class YOLONode(Node, YOLODetector):
    """
    A ROS2 node that performs real-time object detection using YOLOv8 and publishes
    detected bounding boxes, visualized detection images, and provides service/action interfaces.

    ## Parameters:
    - `weights` (string)
    Path to the YOLOv8 model weights file.
    - `camera_topic` (string)
    The input ROS topic for RGB images.
    - `device` (string)
    Device to run inference on. Options: `'cuda'` or `'cpu'`. Defaults to `'cuda'` if available.
    - `conf` (float)
    Confidence threshold for filtering detections.
    - `draw` (bool)
    Whether to draw bounding boxes on the output image.
    - `target_category` (string)
    Target class name to filter detections. If empty, all classes are allowed.

    ## Publishers:
    - `/utbots/vision/detection/image` (sensor_msgs/Image)
    Publishes the image with visualized detections (if drawing is enabled).
    - `/utbots/vision/detection/bounding_boxes` (utbots_msgs/BoundingBoxes)
    Publishes detected bounding boxes with class names and confidence scores.

    ## Subscribers:
    - `<camera_topic>` (sensor_msgs/Image)
    Subscribes to the RGB image stream for inference.

    ## Services:
    - `/utbots/vision/enable_detection` (std_srvs/SetBool)
    Enables or disables synchronous detection processing.

    ## Actions:
    - `YOLO_detection` (utbots_actions/YOLODetection)
    Action server to process a single image and return detection results.
    """
    def __init__(self):
        Node.__init__(self, 'yolo_node')
        
        # Set parameters
        self.declare_parameter('weights', '/ros2_ws/src/yolov8_ros/weights/best.pt')
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('draw', False)
        self.declare_parameter('target_category', '')
        
        self.weights = self.get_parameter('weights').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.conf = self.get_parameter('conf').get_parameter_value().integer_value
        self.draw = self.get_parameter('draw').get_parameter_value().bool_value
        self.target_category = self.get_parameter('target_category').get_parameter_value().string_value
        
        YOLODetector.__init__(self)
        self.get_logger().info(f"YOLOv8 Node initialized with device: {self.device}")
        
        # OpenCV image format conversion
        self.bridge = CvBridge()
        self.cv_img = None

        # Define ROS messages
        
        # Publishers and Subscribers
        self.pub_detection_img = self.create_publisher(Image, "/utbots/vision/detection/image", 10)

        self.pub_bounding_boxes = self.create_publisher(BoundingBoxes, "/utbots/vision/detection/bounding_boxes", 10)
        
        self.sub_frame = self.create_subscription(
            Image,
            self.camera_topic,
            self.callback_img,
            10
        )

        # Service to enable/disable synchronous processing
        self.srv_enable = self.create_service(
            SetBool,
            '/utbots/vision/enable_detection',
            self.enable_detection
        )
        self.enable_synchronous = False
        
        # Action server initialization
        self._action_server = ActionServer(
            self,
            YOLODetection,
            'YOLO_detection',
            self.detection_action
        )
        
        # Timer for synchronous processing
        self.timer = self.create_timer(0.1, self.main_callback)
        
    def callback_img(self, msg):
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def enable_detection(self, request, response):
        self.enable_synchronous = request.data
        response.success = True
        response.message = "Detection enabled" if self.enable_synchronous else "Detection disabled"
        return response

    def plot_bboxes(self, detections, target_category):
        msg_boxes = BoundingBoxes()

        for i in range(len(detections)):
            xyxy = detections.xyxy[i]
            conf = detections.confidence[i]
            cls_id = detections.class_id[i]
            if target_category == "" or self.CLASS_NAMES_DICT[cls_id] == target_category:
                bbox = BoundingBox()
                print(xyxy)
                bbox.id = str(self.CLASS_NAMES_DICT[cls_id])
                bbox.probability = float(conf)
                bbox.xmin = int(xyxy[0])
                bbox.ymin = int(xyxy[1])
                bbox.xmax = int(xyxy[2])
                bbox.ymax = int(xyxy[3])
                msg_boxes.bounding_boxes.append(bbox)
        
        return msg_boxes

    async def detection_action(self, goal_handle):
        self.get_logger().info('Executing YOLO detection action...')
        result = YOLODetection.Result()
        
        if goal_handle.request.image.width != 0:
            image = self.bridge.imgmsg_to_cv2(goal_handle.request.image, "bgr8")
        else:
            image = self.cv_img
            
        target_category = goal_handle.request.target_category
        
        if image is not None:
            
            detections, annotated_img = self.predict_detections(image, self.draw)
            bboxes = self.plot_bboxes(detections, target_category)
            
            result.detected_objects = bboxes

            if self.draw:
                result.labeled_image = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
            
            if target_category != "":
                result.success = Bool()
                result.success.data = len(bboxes.bounding_boxes) > 0
            
            goal_handle.succeed()
            return result
        else:
            goal_handle.abort()
            return result

    def main_callback(self):
        if self.cv_img is not None and self.enable_synchronous:
            start_time = time()

            detections, annotated_img = self.predict_detections(self.cv_img, self.draw)
            bboxes = self.plot_bboxes(detections, self.target_category)
            
            # Calculate FPS
            fps = 1 / (time() - start_time)
            cv2.putText(annotated_img, f'FPS: {int(fps)}', (20,70), 
                      cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
            
            if self.draw:
                self.pub_detection_img.publish(
                    self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8"))
            
            self.pub_bounding_boxes.publish(bboxes)

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
