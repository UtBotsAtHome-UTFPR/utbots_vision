#!/usr/bin/env python3
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

class ObjectDetectionLive(Node):

    def __init__(self):
        super().__init__('yolov8_live')
        
        self.declare_parameter('weights', '/ros2_ws/src/yolov8_ros/weights/best.pt')
        self.declare_parameter('camera_topic', '/image_raw')
        
        self.weights = self.get_parameter('weights').value
        self.camera_topic = self.get_parameter('camera_topic').value
        
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO("yolo11n.pt")
        self.model.fuse()
        self.CLASS_NAMES_DICT = self.model.model.names
        
        self.bridge = CvBridge()
        self.cv_img = None
        self.msg_enable = False
        
        self.pub_detection_img = self.create_publisher(Image, "/utbots/vision/detection/image", 10)
        self.pub_bounding_boxes = self.create_publisher(BoundingBoxes, "/utbots/vision/detection/bounding_boxes", 10)
        
        self.sub_frame = self.create_subscription(
            Image,
            self.camera_topic,
            self.callback_img,
            10
        )
        
        self.srv_enable = self.create_service(
            SetBool,
            '/utbots/vision/enable_detection',
            self.enable_detection
        )
        
        self._action_server = ActionServer(
            self,
            YOLODetection,
            'YOLO_detection',
            self.detection_action
        )
        
        self.timer = self.create_timer(1.0, self.main_callback)
        
        self.get_logger().info(f"YOLOv8 Node initialized with device: {self.device}")

    def callback_img(self, msg):
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def enable_detection(self, request, response):
        self.msg_enable = request.data
        response.success = True
        response.message = "Detection enabled" if self.msg_enable else "Detection disabled"
        return response

    async def detection_action(self, goal_handle):
        self.get_logger().info('Executing YOLO detection action...')
        result = YOLODetection.Result()
        
        if goal_handle.request.image.width != 0:
            image = self.bridge.imgmsg_to_cv2(goal_handle.request.image, "bgr8")
        else:
            image = self.cv_img
            
        target_category = goal_handle.request.target_category
        
        if image is not None:
            results = self.model.predict(image, conf=0.45)
            frame, bboxes = self.plot_bboxes(results, image, target_category)
            
            result.labeled_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            result.detected_objects = bboxes
            
            if target_category != "":
                result.success = Bool()
                result.success.data = len(bboxes.bounding_boxes) > 0
            
            goal_handle.succeed()
            return result
        else:
            goal_handle.abort()
            return result

    def plot_bboxes(self, results, frame, target_category):
        msg_boxes = BoundingBoxes()
        detections = sv.Detections(
            xyxy=results[0].boxes.xyxy.cpu().numpy(),
            confidence=results[0].boxes.conf.cpu().numpy(),
            class_id=results[0].boxes.cls.cpu().numpy().astype(int)
        )

        for (xyxy, _, conf, cls_id) in detections:
            if target_category == "" or self.CLASS_NAMES_DICT[cls_id] == target_category:
                bbox = BoundingBox()
                bbox.class_id = str(self.CLASS_NAMES_DICT[cls_id])
                bbox.probability = float(conf)
                bbox.center.x = (xyxy[0] + xyxy[2]) / 2
                bbox.center.y = (xyxy[1] + xyxy[3]) / 2
                bbox.size_x = xyxy[2] - xyxy[0]
                bbox.size_y = xyxy[3] - xyxy[1]
                msg_boxes.bounding_boxes.append(bbox)

        labels = [f"{self.CLASS_NAMES_DICT[cls_id]} {conf:0.2f}" 
                for _, _, conf, cls_id, _ in detections]
        
        frame = sv.BoxAnnotator().annotate(
            scene=frame,
            detections=detections,
            labels=labels
        )
        
        return frame, msg_boxes

    def main_callback(self):
        if self.cv_img is not None and self.msg_enable:
            start_time = time()
            results = self.model(self.cv_img)
            frame, bboxes = self.plot_bboxes(results, self.cv_img, "")
            
            # Calculate FPS
            fps = 1 / (time() - start_time)
            cv2.putText(frame, f'FPS: {int(fps)}', (20,70), 
                      cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
            
            self.pub_detection_img.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
            self.pub_bounding_boxes.publish(bboxes)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionLive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
