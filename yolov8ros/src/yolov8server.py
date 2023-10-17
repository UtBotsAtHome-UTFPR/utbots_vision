import torch
import numpy as np
import cv2
from time import time
from ultralytics import YOLO
import supervision as sv
import rospy
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBoxes, BoundingBox
from cv_bridge import CvBridge
from yolov8ros.srv import YOLODetection, YOLODetectionResponse

class ObjectDetectionFrame:

    def __init__(self):
          
        # Model configurations
        self.weights = rospy.get_param("yolo_weights", "yolov8n.pt")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu' 
        self.model = self.load_model()
        self.CLASS_NAMES_DICT = self.model.model.names
        
        # Image processing objects
        self.box_annotator = sv.BoxAnnotator(sv.ColorPalette.default(), thickness=3, text_thickness=3, text_scale=1.5)
        self.bridge = CvBridge()
        self.cv_img = None
        
        # Node initialization
        rospy.init_node('yolov8_server', anonymous=True)
        
        self.main()
    
    def handle_detection_request(self, req):
        self.cv_img = self.bridge.imgmsg_to_cv2(req.msg_img, desired_encoding="bgr8")
        results = self.predict(self.cv_img)
        frame = self.plot_bboxes(results, self.cv_img)
     
        req.detection_img = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")
        req.bounding_boxes = self.msg_bounding_boxes
        return req

    def load_model(self):
        model = YOLO(self.weights)  # load a pretrained YOLOv8n model
        model.fuse()
        return model
    
    def predict(self, frame):
        results = self.model(frame) # sends the image to the model and returns results
        return results
    
    def plot_bboxes(self, results, frame):
        self.msg_bounding_boxes = BoundingBoxes()
        
         # Extract detections for each class detected
        for result in results:
            boxes = result.boxes.cpu().numpy()
            class_id = boxes.cls[0]
            conf = boxes.conf[0]
            xyxy = boxes.xyxy[0]
        
        # Setup detections for visualization
        detections = sv.Detections(
                    xyxy=results[0].boxes.xyxy.cpu().numpy(),                   # xmin, ymin, xmax, ymax bounding box coordinates
                    confidence=results[0].boxes.conf.cpu().numpy(),
                    class_id=results[0].boxes.cls.cpu().numpy().astype(int),    # numeric indexes for the infered classes
                    )
        
        for coordinates, _, confidence, class_id, tracker_id in detections:
            # Format custom labels
            self.labels = [f"{self.CLASS_NAMES_DICT[class_id]} {confidence:0.2f}"]
            # Assemble BoudingBox object
            bbox = BoundingBox()
            bbox.Class = self.CLASS_NAMES_DICT[class_id]                        # given the class index, returns the name of the class
            bbox.probability = confidence
            bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax = int(coordinates[0]), int(coordinates[1]), int(coordinates[2]), int(coordinates[3])
            # Append BoundingBox to a BoundingBoxes message
            self.msg_bounding_boxes.bounding_boxes.append(bbox)
        
        # Annotate frame
        frame = self.box_annotator.annotate(scene=frame, detections=detections, labels=self.labels)
    
        return frame
    
    def main(self):
        rospy.loginfo(f"Using Device: {self.device}")
        service = rospy.Service('yolov8_server', YOLODetection, handle_detection_request)
        rospy.spin()

detector = ObjectDetectionFrame()
