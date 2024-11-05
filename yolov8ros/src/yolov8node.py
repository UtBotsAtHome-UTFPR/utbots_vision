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
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge
import actionlib
from utbots_actions.msg import YOLODetectionAction, YOLODetectionResult

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
        self.sub_frame = rospy.Subscriber("/camera/color/image_raw", Image, callback=self.callback_img)
        self.pub_detection_img = rospy.Publisher("/utbots/vision/detection/image", Image, queue_size=1)
        self.pub_bounding_boxes = rospy.Publisher("/utbots/vision/detection/bounding_boxes", BoundingBoxes, queue_size=1)
        self.detect_img_service = rospy.Service('/utbots/vision/enable_detection', SetBool, self.enable_detection)

        # Node initialization
        rospy.init_node('yolov8_live', anonymous=True)
        self.rate = rospy.Rate(1) # 1hz
        # Action server initialization
        self._as = actionlib.SimpleActionServer('YOLO_detection', YOLODetectionAction, self.detection_action, False)
        self._as.start()

        rospy.loginfo("YOLO_detection server up")
        
        self.main()
    
    def callback_img(self, msg):
        self.msg_img = msg
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def enable_detection(self, msg):
        self.msg_enable = msg.data
        self.response = SetBoolResponse()
        self.response.success = True
        return self.response

    def detection_action (self, goal):
        # Optional specific image sent as goal
        if goal.Image.width != 0 and goal.Image.height != 0:
            print( goal.Image.width )
            image = self.bridge.imgmsg_to_cv2(goal.Image, desired_encoding="bgr8")
        else:
        # Topic subscribed image
            image = self.cv_img

        # Target Category will filter bboxes. "" if no target
        target_category = goal.TargetCategory.data
        
        if image is not None:
            results = self.predict(image) # Model output
            frame, bboxes = self.plot_bboxes(results, image, target_category) # Labeled frame and BoundingBoxes msg
            frame = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough") # Convert to img_msg

            # Manage action results
            action_res = YOLODetectionResult()
            action_res.DetectedObjs = bboxes
            action_res.LabeledImage = frame
            action_res.Success = Bool()
            if target_category != "":
                if bboxes.bounding_boxes is not []:
                    action_res.Success.data = True
                else:
                    action_res.Success.data = False

            self.pub_detection_img.publish(frame)
            self.pub_bounding_boxes.publish(self.msg_bounding_boxes)
            self._as.set_succeeded(action_res)
        return []

    def load_model(self):
        model = YOLO(self.weights)  # load a pretrained YOLOv8n model
        model.fuse()
        return model

    def predict(self, frame):
        results = self.model(frame) # sends the image to the model and returns results
        return results
    
    def plot_bboxes(self, results, frame, target_category):
        self.msg_bounding_boxes = BoundingBoxes()
        
        # Setup detections for visualization
        detections = sv.Detections(
                    xyxy=results[0].boxes.xyxy.cpu().numpy(),                   # xmin, ymin, xmax, ymax bounding box coordinates
                    confidence=results[0].boxes.conf.cpu().numpy(),
                    class_id=results[0].boxes.cls.cpu().numpy().astype(int),    # numeric indexes for the infered classes
                    )

        for coordinates, _, confidence, class_id, _ in detections:
                # Filters category if not empty
                if (target_category != "" and self.CLASS_NAMES_DICT[class_id] == target_category) or target_category == "": 
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

        bboxes = self.msg_bounding_boxes
    
        return frame, bboxes
    
    def main(self):
        rospy.loginfo(f"Using Device: {self.device}")
        while not rospy.is_shutdown():
            if self.cv_img is not None and self.msg_enable:
                start_time = time()
                results = self.predict(self.cv_img)
                frame, _ = self.plot_bboxes(results, self.cv_img, "")
                
                # Calculate frames per second
                end_time = time()
                fps = 1/np.round(end_time - start_time, 2)
                cv2.putText(frame, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
                
                self.pub_detection_img.publish(self.bridge.cv2_to_imgmsg(frame, encoding="passthrough"))    # publish image with bounding boxes
                self.pub_bounding_boxes.publish(self.msg_bounding_boxes)                                    # publish detected objects
            self.rate.sleep()
           
detector = ObjectDetectionLive()
