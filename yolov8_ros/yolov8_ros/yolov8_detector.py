#!/usr/bin/env python3
import numpy as np
from ultralytics import YOLO
import supervision as sv
import torch

class YOLODetector():
    """
    A class that processes OpenCV format images using Ultralytics YOLO detection
    and predicts object bounding box in the image.

    ## Parameters:
    - `weights` (string)
    Path of the selected YOLO model. 
    - `device` (str)
    Specifies the device for inference (e.g., cpu, cuda:0 or 0).. 
    - `conf` (float)
    Confidence threshold for detecting a valid bounding box. 
    - `task` (str)
    """
    def __init__(self, weights='yolo11n.pt', device='cuda' if torch.cuda.is_available() else 'cpu', conf=0.25, task="detect"):
        self.weights = weights
        self.device = device
        self.conf = conf
        self.task = task
        self.load_model(
            weights=self.weights,
            task=self.task
        )

    def load_model(self, weights, task):
        """ Loads the YOLO model with the selected parameters"""
        # TODO: task param
        self.model = YOLO(weights)
        self.model.fuse()
        self.CLASS_NAMES_DICT = self.model.model.names

    def unload_model(self):
        """ Unloads the model and stops memory usage """
        if hasattr(self, 'pose'):
            self.model.close()
            del self.model

    def predict_detections(self, cv_image, draw = False):
        # Check if the image is in cv format
        if not isinstance(cv_image, (np.ndarray, np.generic)):
            print(type(cv_image))
            raise ValueError("Input image must be a valid OpenCV image (numpy array).")
        
        # Verify if the model is loaded
        if not hasattr(self, 'model'):
            self.load_model(
                weights=self.weights,
                task=self.task
            )
            raise RuntimeWarning("The pose model is not loaded. Loading now, in runtime.")

        # Predict and format detection
        results = self.model.predict(cv_image, conf=self.conf, device=self.device)
        detections = sv.Detections(
            xyxy=results[0].boxes.xyxy.cpu().numpy(),
            confidence=results[0].boxes.conf.cpu().numpy(),
            class_id=results[0].boxes.cls.cpu().numpy().astype(int)
        )

        # If draw, annotate the cv_image frame
        if draw:
            labels = [f"{self.CLASS_NAMES_DICT[cls_id]} {conf:0.2f}" 
            for _, _, conf, cls_id, _ in detections]
            annotated_img = sv.BoxAnnotator().annotate(
                scene=cv_image,
                detections=detections,
                labels=labels
            )
        else:
            annotated_img = None

        return detections, annotated_img