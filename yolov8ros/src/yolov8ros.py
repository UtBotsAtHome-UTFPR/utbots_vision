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

class ObjectDetection:

    def __init__(self):
       
        self.capture_index = 0
        
        self.weights = rospy.get_param("yolo_weights", "yolov8n.pt")
        
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        self.model = self.load_model()
        
        self.CLASS_NAMES_DICT = self.model.model.names
    
        self.box_annotator = sv.BoxAnnotator(sv.ColorPalette.default(), thickness=3, text_thickness=3, text_scale=1.5)
        
        self.bridge = CvBridge()
        
        self.sub_frame = rospy.Subscriber("/camera/rgb/image_raw", Image, callback=self.callback_img)
        
        self.pub_detection_img = rospy.Publisher("/utbots/vision/detection/image", Image, queue_size=1)
        self.pub_bounding_boxes = rospy.Publisher("/utbots/vision/detection/bounding_boxes", BoundingBoxes, queue_size=1)
        
        self.msg_img = Image()
        self.msg_bounding_boxes = BoundingBoxes()
        self.cv_img = None
        
        rospy.init_node('yolov8_live', anonymous=True)
        self.rate = rospy.Rate(30) # 30hz
        
        self.main()
    
    def callback_img(self, msg):
        self.msg_img = msg
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def load_model(self):
       
        model = YOLO(self.weights)  # load a pretrained YOLOv8n model
        model.fuse()
    
        return model

    def predict(self, frame):
       
        results = self.model(frame)
        
        return results
    
    def plot_bboxes(self, results, frame):
        
        xyxys = []
        confidences = []
        class_ids = []
        self.msg_bounding_boxes = BoundingBoxes()
        
         # Extract detections for person class
        for result in results:
            boxes = result.boxes.cpu().numpy()
            class_id = boxes.cls[0]
            conf = boxes.conf[0]
            xyxy = boxes.xyxy[0]

            if class_id == 0.0:
          
              xyxys.append(result.boxes.xyxy.cpu().numpy())
              confidences.append(result.boxes.conf.cpu().numpy())
              class_ids.append(result.boxes.cls.cpu().numpy().astype(int))
        
        # Setup detections for visualization
        detections = sv.Detections(
                    xyxy=results[0].boxes.xyxy.cpu().numpy(),
                    confidence=results[0].boxes.conf.cpu().numpy(),
                    class_id=results[0].boxes.cls.cpu().numpy().astype(int),
                    )
        
        for coordinates, _, confidence, class_id, tracker_id in detections:
            # Format custom labels
            self.labels = [f"{self.CLASS_NAMES_DICT[class_id]} {confidence:0.2f}"]
            bbox = BoundingBox()
            bbox.Class = self.CLASS_NAMES_DICT[class_id]
            bbox.probability = confidence
            bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax = int(coordinates[0]), int(coordinates[1]), int(coordinates[2]), int(coordinates[3])
            self.msg_bounding_boxes.bounding_boxes.append(bbox)
        
        # Annotate and display frame
        frame = self.box_annotator.annotate(scene=frame, detections=detections, labels=self.labels)
    
        return frame
    
    def main(self):
        rospy.loginfo(f"Using Device: {self.device}")
        while not rospy.is_shutdown():
            if self.cv_img != None:
                results = self.predict(self.cv_img)
                frame = self.plot_bboxes(results, self.cv_img)
                
                end_time = time()
                fps = 1/np.round(end_time - start_time, 2)
                    
                cv2.putText(frame, f'FPS: {int(fps)}', (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
                
                self.pub_detection_img.publish(self.bridge.cv2_to_imgmsg(frame, encoding="passthrough"))
                self.pub_bounding_boxes.publish(self.msg_bounding_boxes)
            self.rate.sleep()
           
detector = ObjectDetection()
