#!/usr/bin/env python3
import numpy as np
from ultralytics import YOLO, SAM
import supervision as sv
import torch
import gc

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
    Task for the model, either "detect", "classify", "segment", or "pose".
    - `segmentation` (bool)
    Whether to enable segmentation alongside detection.
    """
    def __init__(self, weights='yolo11n.pt', device='cuda' if torch.cuda.is_available() else 'cpu', conf=0.25, task="detect", segmentation=True):
        self.weights = weights
        self.device = device
        self.conf = conf
        self.task = task
        self.segmentation = segmentation
        self.load_model(
            weights=self.weights,
            task=self.task
        )
        if self.segmentation:
            self.load_segmentation_model()

    def load_model(self, weights, task = None):
        """ Loads the YOLO model with the selected parameters"""
        # TODO: task param
        self.model = YOLO(weights)
        self.model.fuse()
        self.CLASS_NAMES_DICT = self.model.model.names

    def load_segmentation_model(self, weights="sam2.1_b.pt"):
        """ Loads the YOLO segmentation model"""
        self.segmentation_model = SAM(weights)

    def unload_model(self):
        """ Unloads the model and stops memory usage """
        if hasattr(self, 'model'):
            self.model.to("meta")
            del self.model
            self.model = None

        # Run garbage collector
        gc.collect()

        # Empty PyTorch CUDA cache
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
            torch.cuda.ipc_collect()

    def annotate_image(self, cv_image, detections, labels = []):
        """ Annotate detected bounding boxes with the detected objects """
        if len(labels) <= 0:
            labels = [
                    f"{self.CLASS_NAMES_DICT[class_name]} {confidence:.2f}"
                    for class_name, confidence
                    in zip(detections.class_id, detections.confidence)
                ]
        
        box_annotator = sv.BoxAnnotator(color_lookup=sv.ColorLookup.INDEX)
        label_annotator = sv.LabelAnnotator(color_lookup=sv.ColorLookup.INDEX)

        # First, annotate the boxes
        annotated_img = box_annotator.annotate(
            scene=cv_image.copy(), # It's good practice to work on a copy of the image
            detections=detections
        )

        # Then, annotate the labels on the already annotated image
        annotated_img = label_annotator.annotate(
            scene=annotated_img,
            detections=detections,
            labels=labels # Pass your list of labels here
        )

        return annotated_img

    def predict_detections(self, cv_image, draw = False, disable_segm=False):
        """ Perform predictions of detected objects """
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
            raise RuntimeWarning("The model is not loaded. Loading now, in runtime.")

        # Predict and format detection
        results = self.model.predict(cv_image, conf=self.conf, device=self.device)
        detections = sv.Detections(
            xyxy=results[0].boxes.xyxy.cpu().numpy(),
            confidence=results[0].boxes.conf.cpu().numpy(),
            class_id=results[0].boxes.cls.cpu().numpy().astype(int),
            data={"xyxyn": results[0].boxes.xyxyn.cpu().numpy()}
        )

        # If draw, annotate the cv_image frame
        if draw:
            annotated_img = self.annotate_image(cv_image, detections)
        else:
            annotated_img = cv_image

        if self.segmentation and not disable_segm:
            detections, annotated_img = self.predict_segmentation(cv_image, detections, draw)

        return detections, annotated_img
    
    def predict_segmentation(self, cv_image, detections, draw = False):
        # Predict and format prediction
        sam_results = self.segmentation_model.predict(cv_image, bboxes=detections.xyxy, device=self.device,verbose=False)
        sam_masks = sam_results[0].masks.data.cpu().numpy()
        detections.mask = sam_masks

        # If draw, annotate the cv_image frame
        if draw:
            mask_annotator = sv.MaskAnnotator()

            annotated_img = cv_image.copy()
            
            annotated_img = mask_annotator.annotate(
                scene=annotated_img, 
                detections=detections
            )
        else:
            annotated_img = cv_image

        return detections, annotated_img