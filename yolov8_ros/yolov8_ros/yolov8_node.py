#!/usr/bin/env python3
from .yolov8_detector import YOLODetector
import torch
import cv2
import queue
from typing import List
import supervision as sv
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from utbots_msgs.msg import BoundingBoxes, BoundingBox
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
from utbots_actions.action import YOLODetection, YOLOBatchDetection
from utbots_srvs.srv import LoadModel
from geometry_msgs.msg import Polygon, Point32

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
    - `target_categories` (list of string)
    Target class names to filter detections. If empty, all classes are allowed.

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
        self.declare_parameter('weights', 'yolo11n.pt') # Create a weight folder and put this in it
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('draw', False)
        self.declare_parameter('target_categories', [])
        self.declare_parameter('segmentation', False)
        self.declare_parameter('debug', False)
        self.declare_parameter('enable_synchronous_startup', False)

        self.weights = self.get_parameter('weights').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.conf = self.get_parameter('conf').get_parameter_value().double_value
        self.draw = self.get_parameter('draw').get_parameter_value().bool_value
        self.target_categories = self.get_parameter('target_categories').get_parameter_value().string_array_value
        self.segmentation = self.get_parameter('segmentation').get_parameter_value().bool_value
        self.debug=self.get_parameter('debug').get_parameter_value().bool_value
        self.enable_synchronous =self.get_parameter('enable_synchronous_startup').get_parameter_value().bool_value

        YOLODetector.__init__(self, weights=self.weights, device=self.device, conf=self.conf, segmentation=self.segmentation)
        self.get_logger().info(f"YOLOv8 Node initialized with parameters:\n - Weights: {self.weights}\n - Device: {self.device}\n - Confidence: {self.conf}\n - Segmentation: {self.segmentation}")

        # OpenCV image format conversion
        self.bridge = CvBridge()
        self.cv_img = None
        
        # Publishers and Subscribers
        self.pub_detection_img = self.create_publisher(Image, "/utbots/vision/detection/image", 10)

        self.pub_bounding_boxes = self.create_publisher(BoundingBoxes, "/utbots/vision/detection/bounding_boxes", 10)
        
        self.sub_frame = self.create_subscription(
            Image,
            self.camera_topic,
            self.callback_img,
            10
        )
        self.image_queue = queue.Queue(maxsize=10)

        # Service to enable/disable synchronous processing
        self.srv_enable = self.create_service(
            SetBool,
            '/yolo_node/enable_detection',
            self.enable_detection
        )

        self.srv_load_model = self.create_service(
            LoadModel,
            '/yolo_node/load_model',
            self.load_model_cb
        )
        
        # Action server initialization
        self._action_server = ActionServer(
            self,
            YOLODetection,
            'YOLO_detection',
            self.detection_action
        )

        self._batch_action_server = ActionServer(
            self,
            YOLOBatchDetection,
            'YOLO_batch_detection',
            self.batch_detection_action
        )
        
        # Timer for synchronous processing
        self.timer = self.create_timer(0.1, self.main_callback)

        
    def callback_img(self, msg):
        """ RBG image topic callback """
        if(self.debug):
            self.get_logger().info(f"[YOLO] Callback image")
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        try:
            self.image_queue.put_nowait(self.cv_img)
        except queue.Full:
            # drop the oldest image if queue is full
            _ = self.image_queue.get_nowait()
            self.image_queue.put_nowait(self.cv_img)

    def enable_detection(self, request, response):
        """ Service callback for synchronous detections """
        self.enable_synchronous = request.data
        response.success = True
        response.message = "Detection enabled" if self.enable_synchronous else "Detection disabled"
        return response
    
    def load_model_cb(self, request, response):
        """ Load/Reload YOLO model """
        # TODO: make this work
        if request.data == "":
            self.enable_synchronous = False
            time.sleep(0.2)
            self.unload_model()
            return response

        reactivate = self.enable_synchronous

        self.enable_synchronous = False

        time.sleep(0.2)

        if request.data == "coco":
            self.load_model(str(Model.coco.value))
        elif request.data == "trained":
            self.load_model(str(Model.trained))
        else:
            self.weights = request.data
        
        self.enable_synchronous = reactivate

        response.success = True
        return response

    def format_bbox_msg(self, detections, target_categories):
        """ Format ROS bounding box messages """
        msg_boxes = BoundingBoxes()
        if detections is None:
            return msg_boxes
        for i in range(len(detections)):
            xyxy = detections.xyxy[i]
            xyxyn = detections.data["xyxyn"][i]
            conf = detections.confidence[i]
            cls_id = detections.class_id[i]
            if not target_categories or self.CLASS_NAMES_DICT[cls_id] in [s.data for s in target_categories]:
                bbox = BoundingBox()
                bbox.id = int(cls_id)
                bbox.category = str(self.CLASS_NAMES_DICT[cls_id]) 
                bbox.probability = float(conf)
                bbox.xmin = int(xyxy[0])
                bbox.ymin = int(xyxy[1])
                bbox.xmax = int(xyxy[2])
                bbox.ymax = int(xyxy[3])
                bbox.xminn = float(xyxyn[0])
                bbox.yminn = float(xyxyn[1])
                bbox.xmaxn = float(xyxyn[2])
                bbox.ymaxn = float(xyxyn[3])
                if self.segmentation and hasattr(detections, "mask") and len(detections.mask) > i:
                    bbox.mask = self.format_mask_msg(detections.mask[i])
                msg_boxes.bounding_boxes.append(bbox)
        return msg_boxes
    
    def format_mask_msg(self, detections):
        """Format segmentation mask as a binary image message."""
        # detections here is a single mask (np.ndarray), not a Detections object
        if detections is None:
            return None

        # Ensure mask is uint8 and binary (0 or 1)
        mask = (detections > 0.5).astype(np.uint8)

        # Convert to ROS Image message
        mask_msg = self.bridge.cv2_to_imgmsg(mask * 255, encoding="mono8")
        return mask_msg

    def detection_action(self, goal_handle):
        """ Single detection action callback"""
        self.get_logger().info('Executing YOLO detection action...')
        result = YOLODetection.Result()
        
        if goal_handle.request.image.width != 0:
            image = self.bridge.imgmsg_to_cv2(goal_handle.request.image, "bgr8")
        else:
            image = self.cv_img
            
        target_categories = goal_handle.request.target_categories
        
        if image is not None:
            
            detections, annotated_img = self.predict_detections(image, self.draw)
            bboxes = self.format_bbox_msg(detections, target_categories)
            
            result.detected_objs = bboxes

            if self.draw:
                result.labeled_image = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
                self.pub_detection_img.publish(result.labeled_image)
            
            if target_categories != []:
                result.success = Bool()
                result.success.data = len(bboxes.bounding_boxes) > 0
            
            goal_handle.succeed()
            return result
        else:
            goal_handle.abort()
            return result

    def compute_iou(self, box1, box2):
        """ Calculate Intersecction Over Union between bounding boxes """
        # box: [xmin, ymin, xmax, ymax]
        xA = max(box1.xmin, box2.xmin)
        yA = max(box1.ymin, box2.ymin)
        xB = min(box1.xmax, box2.xmax)
        yB = min(box1.ymax, box2.ymax)

        interW = max(0, xB - xA)
        interH = max(0, yB - yA)
        interArea = interW * interH

        box1Area = (box1.xmax - box1.xmin) * (box1.ymax - box1.ymin)
        box2Area = (box2.xmax - box2.xmin) * (box2.ymax - box2.ymin)

        unionArea = box1Area + box2Area - interArea
        if unionArea == 0:
            return 0.0
        return interArea / unionArea

    def batch_detection_action(self, goal_handle):
        """ Batch Detection Action Callback """
        self.get_logger().info('Executing YOLO detection batch action...')
        result = YOLOBatchDetection.Result()

        batch_size = goal_handle.request.batch_size.data
        target_categories = goal_handle.request.target_categories
        self.get_logger().info(f"Target categories{target_categories}")
        iou_thresh = goal_handle.request.iou_threshold.data
        support_threshold = goal_handle.request.support_threshold.data  

        self.count_batch = True
        self.bboxes = BoundingBoxes()
        
        for i in range(batch_size):
            print(i)
            try:
                image = self.image_queue.get(timeout=2.0)  # wait up to 2s
            except queue.Empty:
                self.get_logger().warn("No image received in time")
                goal_handle.abort()
                return result

            detections, annotated_img = self.predict_detections(image, False, True)
            if detections is None:
                self.get_logger().warn(f"No detections on batch index {i}")
                continue
            # Convert detections to sv.Detections object for merging
            if detections is None or len(detections.xyxy) == 0:
                self.get_logger().warn(f"No valid detections on batch index {i}")
                continue

            print(detections)

            # For the first image, initialize aggregation
            if i == 0:
                aggregated_detections = detections
                contributor_counts = [1] * len(detections.xyxy)
            else:
                # For each new detection, try to match with existing aggregated detections
                new_xyxy = detections.xyxy
                new_xyxyn = detections.data["xyxyn"]
                new_conf = detections.confidence
                new_class_id = detections.class_id

                agg_xyxy = aggregated_detections.xyxy
                agg_xyxyn = aggregated_detections.data["xyxyn"]
                agg_conf = aggregated_detections.confidence
                agg_class_id = aggregated_detections.class_id

                # Prepare lists for updated aggregation
                updated_xyxy = []
                updated_xyxyn = []
                updated_conf = []
                updated_class_id = []
                updated_counts = []

                matched_indices = set()
                for idx_new, (bbox_new, bboxn_new, class_new) in enumerate(zip(new_xyxy, new_xyxyn, new_class_id)):
                    best_iou = 0.0
                    best_idx = -1
                    for idx_agg, (bbox_agg, bboxn_agg, class_agg) in enumerate(zip(agg_xyxy, agg_xyxyn, agg_class_id)):
                        if class_new != class_agg:
                            continue
                        # Compute IoU
                        xA = max(bbox_new[0], bbox_agg[0])
                        yA = max(bbox_new[1], bbox_agg[1])
                        xB = min(bbox_new[2], bbox_agg[2])
                        yB = min(bbox_new[3], bbox_agg[3])
                        interW = max(0, xB - xA)
                        interH = max(0, yB - yA)
                        interArea = interW * interH
                        area_new = (bbox_new[2] - bbox_new[0]) * (bbox_new[3] - bbox_new[1])
                        area_agg = (bbox_agg[2] - bbox_agg[0]) * (bbox_agg[3] - bbox_agg[1])
                        unionArea = area_new + area_agg - interArea
                        iou = interArea / unionArea if unionArea > 0 else 0.0
                        if iou > best_iou:
                            best_iou = iou
                            best_idx = idx_agg
                    if best_iou > iou_thresh and best_idx != -1:
                        # Merge: average coordinates, keep max conf, increment count
                        merged_xyxy = (agg_xyxy[best_idx] + bbox_new) / 2.0
                        merged_xyxyn = (agg_xyxyn[best_idx] + bboxn_new) / 2.0
                        merged_conf = max(agg_conf[best_idx], new_conf[idx_new])
                        merged_class = class_new
                        merged_count = contributor_counts[best_idx] + 1
                        updated_xyxy.append(merged_xyxy)
                        updated_xyxyn.append(merged_xyxyn)
                        updated_conf.append(merged_conf)
                        updated_class_id.append(merged_class)
                        updated_counts.append(merged_count)
                        matched_indices.add(best_idx)
                    else:
                        # New detection, add as is
                        updated_xyxy.append(bbox_new)
                        updated_xyxyn.append(bboxn_new)
                        updated_conf.append(new_conf[idx_new])
                        updated_class_id.append(class_new)
                        updated_counts.append(1)
                # Add unmatched previous aggregated detections
                for idx_agg, (bbox_agg, bboxn_agg, conf_agg, class_agg, count_agg) in enumerate(
                    zip(agg_xyxy, agg_xyxyn, agg_conf, agg_class_id, contributor_counts)
                ):
                    if idx_agg not in matched_indices:
                        updated_xyxy.append(bbox_agg)
                        updated_xyxyn.append(bboxn_agg)
                        updated_conf.append(conf_agg)
                        updated_class_id.append(class_agg)
                        updated_counts.append(count_agg)
                # Update aggregation
                aggregated_detections = sv.Detections(
                    xyxy=np.array(updated_xyxy, dtype=np.float32),
                    data={"xyxyn": np.array(updated_xyxyn, dtype=np.float32)},
                    confidence=np.array(updated_conf, dtype=np.float32),
                    class_id=np.array(updated_class_id, dtype=np.int64)
                )
                contributor_counts = updated_counts

        # ----------------------------------------------------- #
        # Post‑processing: keep only detections with support >= support_threshold
        norm_counts = [c / batch_size for c in contributor_counts]
        filtered_indices = [idx for idx, contrib in enumerate(norm_counts) if contrib >= support_threshold]
        if len(filtered_indices) > 0:
            filtered_detections = sv.Detections(
                xyxy=aggregated_detections.xyxy[filtered_indices],
                data={"xyxyn": aggregated_detections.data["xyxyn"][filtered_indices]},
                confidence=aggregated_detections.confidence[filtered_indices],
                class_id=aggregated_detections.class_id[filtered_indices]
            )
        else:
            filtered_detections = sv.Detections(
                xyxy=np.zeros((0, 4), dtype=np.float32),
                data={"xyxyn": np.zeros((0, 4), dtype=np.float32)},
                confidence=np.zeros((0,), dtype=np.float32),
                class_id=np.zeros((0,), dtype=np.int64)
            )
        # Convert to BoundingBoxes message
        annotated_img = self.annotate_image(self.image_queue.get(timeout=2.0), filtered_detections)

        if self.segmentation:
            filtered_detections, annotated_img = self.predict_segmentation(annotated_img, filtered_detections, self.draw)
        
        filtered_bboxes = self.format_bbox_msg(filtered_detections, target_categories)
        annotated_img_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
        result.annotated_image = annotated_img_msg
        result.detected_objs = filtered_bboxes

        # Publish image for visualization
        if self.draw:
            self.pub_detection_img.publish(annotated_img_msg)

        goal_handle.succeed()
        return result

    def main_callback(self):
        """ Synchronous processing callback """
        if self.cv_img is not None and self.enable_synchronous:
            start_time = time.time()

            detections, annotated_img = self.predict_detections(self.cv_img, self.draw)
            bboxes = self.format_bbox_msg(detections, self.target_categories)
            
            # Calculate FPS
            fps = 1 / (time.time() - start_time)
            cv2.putText(annotated_img, f'FPS: {int(fps)}', (20,70), 
                      cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 2)
            
            if self.draw:
                self.pub_detection_img.publish(
                    self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8"))
            self.pub_bounding_boxes.publish(bboxes)

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
