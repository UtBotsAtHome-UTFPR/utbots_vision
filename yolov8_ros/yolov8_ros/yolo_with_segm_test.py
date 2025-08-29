# Import necessary libraries
from ultralytics import YOLO, SAM
import supervision as sv
import cv2
import numpy as np
import os

# Load models and image (assuming IMG_SOURCE is defined)
HOME = os.getcwd()
IMAGE_NAME = HOME + 'Downloads/MundialDrinks/tangerine.jpg'
img = cv2.imread(IMAGE_NAME)
yolo_model = YOLO('yolov8n.pt')
sam_model = SAM("sam2.1_b.pt")

# YOLO prediction
results = yolo_model.predict(source=img, verbose=False)
yolo_boxes = results[0].boxes.xyxy
yolo_classes = results[0].boxes.cls
yolo_confidences = results[0].boxes.conf

# SAM prediction
sam_results = sam_model.predict(img, bboxes=yolo_boxes.cpu().numpy(), verbose=False)
sam_masks = sam_results[0].masks.data.cpu().numpy()

# Create Detections object
detections = sv.Detections(
    xyxy=yolo_boxes.cpu().numpy(),
    mask=sam_masks,
    class_id=yolo_classes.cpu().numpy().astype(int),
    confidence=yolo_confidences.cpu().numpy(),
)

# Initialize and use annotators
box_annotator = sv.BoxAnnotator()
mask_annotator = sv.MaskAnnotator()
annotated_img = img.copy()

annotated_img = mask_annotator.annotate(
    scene=annotated_img, 
    detections=detections
)
annotated_img = box_annotator.annotate(
    scene=annotated_img, 
    detections=detections
)

# Plot the image with a defined size to avoid the error
sv.plot_image(annotated_img)