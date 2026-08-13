import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import math
import time
from lidar_tracking.sort import Sort


# New imports for TF2 Broadcasting 
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster

from rclpy.action import ActionServer
from utbots_actions.action import TrackPerson

from rclpy.executors import MultiThreadedExecutor

class LidarTrackingNode(Node):

    def __init__(self):
        super().__init__('lidar_tracking_node')

        # Action state
        self.action_active = False
        self.target_direction = None
        self.tracked_id = -1
        self.tracking_state = "WAITING"

        # Action Server
        self.action_server = ActionServer(
            self,
            TrackPerson,
            'track_person',
            self.execute_callback
        )

        # CONFIGURATION PARAMETERS 

        self.declare_parameter('lidar_topic', '/scan')
        self.lidar_topic = self.get_parameter(
            'lidar_topic'
        ).get_parameter_value().string_value

        # 1. Image Settings
        self.img_size = 240
        self.center = self.img_size // 2
        self.resolution = 0.05

        # 2. Morphology and Size Filter Settings 
        self.kernel_size = 3

        self.min_width = 2
        self.max_width = 8
        self.min_height = 2
        self.max_height = 8

        # 3. Padding 
        self.padding = 10

        # 4. SORT Tracker Settings
        self.max_age = 5
        self.min_hits = 1
        self.iou_threshold = 0.2

        # LaserScan subscription
        self.subscription = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.scan_callback,
            10
        )

        self.tracker = Sort(
            max_age=self.max_age,
            min_hits=self.min_hits,
            iou_threshold=self.iou_threshold
        )

        # TF2 Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'Lidar Tracking Action Server started.'
        )

    def execute_callback(self, goal_handle):

        self.action_active = True
        self.target_direction = goal_handle.request.initial_direction
        self.tracked_id = -1
        self.tracking_state = "WAITING"

        feedback_msg = TrackPerson.Feedback()

        while self.action_active:

            if goal_handle.is_cancel_requested:
                self.action_active = False
                goal_handle.canceled()
                return TrackPerson.Result(success=False)

        #waits scan_callback tracking update

            feedback_msg.tracked_id = int(self.tracked_id)
            feedback_msg.x = 0.0
            feedback_msg.y = 0.0
            feedback_msg.state = self.tracking_state

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)

        goal_handle.succeed()

        result = TrackPerson.Result()
        result.success = True
        result.tracked_id = self.tracked_id

        return result


    def scan_callback(self, msg):

        if not self.action_active:
            return
        
        # RASTERIZATION (Lidar to Pixels)
        img = np.zeros((self.img_size, self.img_size), dtype=np.uint8)
        for i, range_val in enumerate(msg.ranges):
            if np.isinf(range_val) or math.isnan(range_val) or range_val > msg.range_max or range_val < msg.range_min:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)

            pixel_x = self.center - int(y / self.resolution)
            pixel_y = self.center - int(x / self.resolution)

            if 0 <= pixel_x < self.img_size and 0 <= pixel_y < self.img_size:
                img[pixel_y, pixel_x] = 255

        # PRE-PROCESSING (Morphological Closing)
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        
        # DETECTION AND FILTERING
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for idx, contour in enumerate(contours):
            x, y, w, h = cv2.boundingRect(contour)

            # Filter: Only accepts if it is WITHIN the size window
            if (self.min_width <= w <= self.max_width) and (self.min_height <= h <= self.max_height):
                cv2.rectangle(img_color, (x, y), (x + w, y + h), (0, 255, 0), 1)

                # Applies PADDING for SORT
                # Here we extend the size of the bounding boxes to improve IoU association between consecutive frames.
                x1_pad = x - self.padding
                y1_pad = y - self.padding
                x2_pad = x + w + self.padding
                y2_pad = y + h + self.padding
                
                detections.append([x1_pad, y1_pad, x2_pad, y2_pad])
                
            else:
                cv2.rectangle(img_color, (x, y), (x + w, y + h), (0, 0, 255), 1) # Draws red for debugging

        # TRACKING (SORT)
        if len(detections) > 0:
            np_detections = np.asarray(detections, dtype=float).reshape(-1, 4)
        else:
            np_detections = np.empty((0, 4))
            
        tracked_objects = self.tracker.update(np_detections)
        
        # VISUALIZATION, DE-RASTERIZATION AND TF2 BROADCASTING
        
        for obj in tracked_objects:

            #if obj_id != self.tracked_id:
            #    continue


            x1, y1, x2, y2, obj_id = map(int, obj)
            
            # Calculates the center in pixels. Note that this center is from the padded bounding box
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # OpenCV Drawing
            cv2.rectangle(img_color, (x1, y1), (x2, y2), (0, 255, 255), 1)
            cv2.circle(img_color, (cx, cy), 3, (0, 255, 255), -1)
            cv2.putText(img_color, f"ID: {obj_id}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            # DE-RASTERIZATION: Pixels to Meters
            # Attention to the axis swap: Lidar X is OpenCV Y (up/down)
            pos_x = (self.center - cy) * self.resolution
            pos_y = (self.center - cx) * self.resolution

            
            # DYNAMIC FRAME PUBLICATION 
            
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            
            # The parent frame is the sensor itself (e.g., 'base_scan')
            t.header.frame_id = msg.header.frame_id
            
            # The child frame represents the tracked object dynamically
            t.child_frame_id = f"lidar_leg_{obj_id}"

            # Local position directly from the de-rasterization math
            t.transform.translation.x = float(pos_x)
            t.transform.translation.y = float(pos_y)
            t.transform.translation.z = 0.0

            # Neutral rotation (Cylinders/Legs do not have a specific facing direction here)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Send the transform to the TF2 Tree
            self.tf_broadcaster.sendTransform(t)

        
        
        img_display = cv2.resize(img_color, (self.img_size * 2, self.img_size * 2), interpolation=cv2.INTER_NEAREST)
        cv2.imshow('LIDAR Tracking', img_display)
        cv2.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)
    node = LidarTrackingNode()
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()