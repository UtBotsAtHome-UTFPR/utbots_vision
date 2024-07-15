#!/usr/bin/env python3
import torch
import numpy as np
import cv2
from time import time
from ultralytics import YOLO
import supervision as sv
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBoxes, BoundingBox
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from cv_bridge import CvBridge

class ObjectDetectionLive:

    def __init__(self):
        # Model configurations
        self.weights = rospy.get_param("yolo_weights", "yolov8n.pt")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu' 
        self.model = self.load_model()
        self.CLASS_NAMES_DICT = self.model.model.names
        
        # Image processing objects
        self.box_annotator = sv.BoxAnnotator(sv.ColorPalette.default(), thickness=2, text_thickness=1, text_scale=0.75)
        self.bridge = CvBridge()
        self.cv_img = None
        self.msg_enable = False
        
        # Messages
        self.msg_img = Image()
        self.msg_bounding_boxes = BoundingBoxes()
        
        # Subscribers and Publishers and Services
        self.sub_frame = rospy.Subscriber("/usb_cam/image_raw", Image, callback=self.callback_img)
        self.sub_enable = rospy.Subscriber("/utbots/vision/detection/enable", Bool, callback=self.callback_enable)
        self.pub_detection_img = rospy.Publisher("/utbots/vision/detection/image", Image, queue_size=1)
        self.pub_bounding_boxes = rospy.Publisher("/utbots/vision/detection/bounding_boxes", BoundingBoxes, queue_size=1)
        self.detect_img_service = rospy.Service('/utbots/vision/detection_srv', Empty, self.detection_srv)

        # Node initialization
        rospy.init_node('yolov8_live', anonymous=True)
        self.rate = rospy.Rate(30) # 30hz
        
        self.main()
    
    def callback_img(self, msg):
        self.msg_img = msg
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def callback_enable(self, msg):
        self.msg_enable = msg.data

    def detection_srv(self, msg):
        if self.cv_img is not None:
            results = self.predict(self.cv_img)
            frame = self.plot_bboxes(results, self.cv_img)
            self.pub_detection_img.publish(self.bridge.cv2_to_imgmsg(frame, encoding="passthrough"))
            self.pub_bounding_boxes.publish(self.msg_bounding_boxes)
        return []

    def load_model(self):
        model = YOLO(self.weights)  # load a pretrained YOLOv8n model
        model.fuse()
        return model

    def predict(self, frame):
        results = self.model(frame) # sends the image to the model and returns results
        return results
    
    def plot_bboxes(self, results, frame):
        self.msg_bounding_boxes = BoundingBoxes()
        
        # Setup detections for visualization
        detections = sv.Detections(
                    xyxy=results[0].boxes.xyxy.cpu().numpy(),                   # xmin, ymin, xmax, ymax bounding box coordinates
                    confidence=results[0].boxes.conf.cpu().numpy(),
                    class_id=results[0].boxes.cls.cpu().numpy().astype(int),    # numeric indexes for the infered classes
                    )

        for coordinates, _, confidence, class_id, _ in detections:
            # Assemble BoudingBox object
            bbox = BoundingBox()
            bbox.Class = self.CLASS_NAMES_DICT[class_id]                        # given the class index, returns the name of the class
            bbox.probability = confidence
            bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax = int(coordinates[0]), int(coordinates[1]), int(coordinates[2]), int(coordinates[3])
            # Append BoundingBox to a BoundingBoxes message
            self.msg_bounding_boxes.bounding_boxes.append(bbox)

        self.labels = [f"{self.CLASS_NAMES_DICT[class_id]} {confidence:0.2f}"
        for _, _, confidence, class_id, _
        in detections]
        
        # Annotate frame
        frame = self.box_annotator.annotate(scene=frame, detections=detections, labels=self.labels)
    
        return frame
    
    def main(self):
        rospy.loginfo(f"Using Device: {self.device}")
        while not rospy.is_shutdown():
            if self.cv_img is not None and self.msg_enable:
                start_time = time()
                results = self.predict(self.cv_img)
                frame = self.plot_bboxes(results, self.cv_img)
                
                # Calculate frames per second
                end_time = time()
                fps = 1/np.round(end_time - start_time, 2)
                cv2.putText(frame, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
                
                self.pub_detection_img.publish(self.bridge.cv2_to_imgmsg(frame, encoding="passthrough"))    # publish image with bounding boxes
                self.pub_bounding_boxes.publish(self.msg_bounding_boxes)                                    # publish detected objects
            self.rate.sleep()
           
detector = ObjectDetectionLive()
