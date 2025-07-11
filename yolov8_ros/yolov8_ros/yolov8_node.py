#!/usr/bin/env python3
from .yolov8_detector import YOLODetector
import torch
import cv2
import queue
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

from enum import Enum

class Model(Enum):
    coco = '~/ros2_ws/src/utbots_vision/yolov8_ros/yolo11n.pt' # Put this in a weight folder in the future
    trained = "/path/to/trained/model"


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
        self.declare_parameter('weights', str(Model.coco)) # Create a weight folder and put this in it
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('draw', False)
        self.declare_parameter('target_categories', [])
        self.declare_parameter('debug', False)
        self.declare_parameter('enable_synchronous_startup', False)

        # self.declare_parameter('callback in ms', False)

        self.weights = self.get_parameter('weights').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.conf = self.get_parameter('conf').get_parameter_value().integer_value
        self.draw = self.get_parameter('draw').get_parameter_value().bool_value
        self.target_categories = self.get_parameter('target_categories').get_parameter_value().string_array_value
        self.debug=self.get_parameter('debug').get_parameter_value().bool_value
        self.enable_synchronous =self.get_parameter('enable_synchronous_startup').get_parameter_value().bool_value

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

        self.count_batch = False
        self.got_image = False
        
    def callback_img(self, msg):
        self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if(self.debug):
            self.get_logger().info(f"[YOLO] Callback image")
        if self.count_batch:
            self.got_image = True

    def enable_detection(self, request, response):
        self.enable_synchronous = request.data
        response.success = True
        response.message = "Detection enabled" if self.enable_synchronous else "Detection disabled"
        return response
    
    def load_model_cb(self, request, response):
        print(Model.coco.value)
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
        #print(self.weights)
        
        self.enable_synchronous = reactivate

        response.success = True
        return response

    def format_bbox_msg(self, detections, target_categories):
        msg_boxes = BoundingBoxes()

        for i in range(len(detections)):
            xyxy = detections.xyxy[i]
            xyxyn = detections.data["xyxyn"][i]
            conf = detections.confidence[i]
            cls_id = detections.class_id[i]
            if not target_categories or self.CLASS_NAMES_DICT[cls_id] in [s.data for s in target_categories]:
                bbox = BoundingBox()
                bbox.id = str(self.CLASS_NAMES_DICT[cls_id])
                bbox.probability = float(conf)
                bbox.xmin = int(xyxy[0])
                bbox.ymin = int(xyxy[1])
                bbox.xmax = int(xyxy[2])
                bbox.ymax = int(xyxy[3])
                bbox.xminn = float(xyxyn[0])
                bbox.yminn = float(xyxyn[1])
                bbox.xmaxn = float(xyxyn[2])
                bbox.ymaxn = float(xyxyn[3])
                msg_boxes.bounding_boxes.append(bbox)
        return msg_boxes

    async def detection_action(self, goal_handle):
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
            
            result.detected_objects = bboxes

            if self.draw:
                result.labeled_image = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
            
            if target_categories != []:
                result.success = Bool()
                result.success.data = len(bboxes.bounding_boxes) > 0
            
            goal_handle.succeed()
            return result
        else:
            goal_handle.abort()
            return result

    async def batch_detection_action(self, goal_handle):
        self.get_logger().info('Executing YOLO detection batch action...')
        result = YOLOBatchDetection.Result()
        batch_size = goal_handle.request.batch_size.data
        target_categories = goal_handle.request.target_categories
        iou_threshold = goal_handle.request.iou_threshold.data
        support_threshold = goal_handle.request.support_threshold.data
        self.count_batch = True
        self.bboxes = BoundingBoxes()
        bbox_contributors = []
        self.get_logger().info(f"Target categories{target_categories}")
        try:
            if self.got_image:
                image = self.cv_img
                self.bboxes = self.format_bbox_msg(detections, target_categories)
                bbox_contributors = [1 for _ in self.bboxes.bounding_boxes]

            def compute_iou(box1, box2):
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

            i = 0     
            while(i < batch_size):
                if self.got_image:
                    print(i)
                    image = self.cv_img
                            
                    if image is not None:
                        detections, annotated_img = self.predict_detections(image, False)
                        bboxes = self.format_bbox_msg(detections, target_categories)

                        for bbox in bboxes.bounding_boxes:
                            max_iou = 0.0
                            max_idx = -1
                            for idx, ref_bbox in enumerate(self.bboxes.bounding_boxes):
                                iou = compute_iou(bbox, ref_bbox)
                                if iou > max_iou:
                                    max_iou = iou
                                    max_idx = idx
                            if max_iou > iou_threshold and max_idx != -1:
                                # Update bbox in self.bboxes with mean coordinates
                                ref_bbox = self.bboxes.bounding_boxes[max_idx]
                                n = bbox_contributors[max_idx]
                                ref_bbox.xmin = int((ref_bbox.xmin + bbox.xmin) / 2)
                                ref_bbox.ymin = int((ref_bbox.ymin + bbox.ymin) / 2)
                                ref_bbox.xmax = int((ref_bbox.xmax + bbox.xmax) / 2)
                                ref_bbox.ymax = int((ref_bbox.ymax + bbox.ymax) / 2)
                                ref_bbox.xminn = float((ref_bbox.xminn + bbox.xminn) / 2)
                                ref_bbox.yminn = float((ref_bbox.yminn + bbox.yminn) / 2)
                                ref_bbox.xmaxn = float((ref_bbox.xmaxn + bbox.xmaxn) / 2)
                                ref_bbox.ymaxn = float((ref_bbox.ymaxn + bbox.ymaxn) / 2)
                                ref_bbox.id = bbox.id
                                bbox_contributors[max_idx] += 1
                            else:
                                # Add new bbox and initialize its contributors count
                                self.bboxes.bounding_boxes.append(bbox)
                                bbox_contributors.append(1)
                    i += 1
                    self.got_image = False
                
            self.count_batch = False
            bbox_contributors = [c / batch_size for c in bbox_contributors]

            # Remove bboxes with contributors less than support_threshold
            filtered_bboxes = BoundingBoxes()
            filtered_contributors = []
            for bbox, contrib in zip(self.bboxes.bounding_boxes, bbox_contributors):
                if contrib >= support_threshold:
                    filtered_bboxes.bounding_boxes.append(bbox)
                    filtered_contributors.append(contrib)

            xyxy_list = []
            conf_list = []
            labels = []

            if len(filtered_bboxes.bounding_boxes) > 0:
                for bbox in filtered_bboxes.bounding_boxes:
                # Prepare [xmin, ymin, xmax, ymax]
                    xyxy_list.append([bbox.xmin, bbox.ymin, bbox.xmax, bbox.ymax])
                    conf_list.append(bbox.probability)
                    # Assign numeric class ID
                    class_name = bbox.id

                    labels.append(f"{class_name} {bbox.probability:.2f}")

                # Create Detections object (this part is correct)
                detections = sv.Detections(
                    xyxy=np.array(xyxy_list, dtype=np.float32),
                    confidence=np.array(conf_list, dtype=np.float32)
                )

                box_annotator = sv.BoxAnnotator(color_lookup=sv.ColorLookup.INDEX)
                label_annotator = sv.LabelAnnotator(color_lookup=sv.ColorLookup.INDEX)

                # First, annotate the boxes
                annotated_img = box_annotator.annotate(
                    scene=self.cv_img.copy(), # It's good practice to work on a copy of the image
                    detections=detections
                )

                # Then, annotate the labels on the already annotated image
                annotated_img = label_annotator.annotate(
                    scene=annotated_img,
                    detections=detections,
                    labels=labels # Pass your list of labels here
                )

                self.pub_detection_img.publish(
                        self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8"))

            result.detected_objs = filtered_bboxes
            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f"Error during batch detection: {str(e)}")
            goal_handle.abort()
            return result

    def main_callback(self):
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
