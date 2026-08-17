import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import math
import time

from lidar_tracking.lidar_processor import LidarProcessor

# New imports for TF2 Broadcasting 
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster

from rclpy.action import ActionServer
from utbots_actions.action import TrackPerson

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class LidarTrackingNode(Node):

    def __init__(self):
        super().__init__('lidar_tracking_node')

        # Action state
        self.action_active = False
        self.target_direction = None
        self.tracked_id = -1
        self.tracking_state = "WAITING"

        # Current Sort trackers
        self.current_tracks = {}

        # Target selection parameter
        self.max_angle_error = math.radians(20.0)

        # Protect shared tracker state between callbacks
        from threading import Lock
        self.tracker_lock = Lock()

        # Multithreading
        self.scan_group = MutuallyExclusiveCallbackGroup()
        self.action_group = MutuallyExclusiveCallbackGroup()

        # Action Server
        self.action_server = ActionServer(
            self,
            TrackPerson,
            'track_person',
            self.execute_callback,
            callback_group=self.action_group
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
            10,
            callback_group=self.scan_group
        )

        self.lidar_processor = LidarProcessor(
            img_size=self.img_size,
            resolution=self.resolution,
            kernel_size=self.kernel_size,
            min_width=self.min_width,
            max_width=self.max_width,
            min_height=self.min_height,
            max_height=self.max_height,
            padding=self.padding,
            max_age=self.max_age,
            min_hits=self.min_hits,
            iou_threshold=self.iou_threshold
        )

        
        # TF2 Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'Lidar Tracking Action Server started.'
        )

    def find_target_by_direction(self, target_direction):
        """
        Selects the current SORT tracker whose angular position is
        closest to the requested target direction.
        """

        best_id = -1
        best_error = float('inf')


        self.get_logger().info(
            f"Target direction: {math.degrees(target_direction):.2f} deg "
            f"({target_direction:.4f} rad)"
        )


        with self.tracker_lock:
            tracks = self.current_tracks.copy()

        for obj_id, track in tracks.items():

            cluster_angle = track["angle"]

            # Smallest angular difference considering the [-pi, pi] wrap-around.
            angle_error = abs(
                (cluster_angle - target_direction + math.pi) %
                (2 * math.pi) -
                math.pi
            )

            self.get_logger().info(
                f"ID={obj_id} "
                f"angle={math.degrees(cluster_angle):.1f} deg "
                f"error={math.degrees(angle_error):.1f} deg"
            )

            if angle_error < best_error:
                best_error = angle_error
                best_id = obj_id

        if best_error > self.max_angle_error:
            self.get_logger().info(
                "No tracker found within the allowed angular range."
            )
            return -1

        self.get_logger().info(
            f"Selected tracker ID={best_id} "
            f"with angular error={math.degrees(best_error):.1f} deg"
        )

        return best_id


    def execute_callback(self, goal_handle):
        
        self.action_active = True
        self.target_direction = goal_handle.request.initial_direction
        self.tracked_id = -1
        self.tracking_state = "ACQUIRING"

        feedback_msg = TrackPerson.Feedback()

        while self.action_active:

            if goal_handle.is_cancel_requested:
                self.action_active = False
                self.tracked_id = -1
                self.tracking_state = "WAITING"

                goal_handle.canceled()

                result = TrackPerson.Result()
                result.success = False
                result.tracked_id = -1

                return result

            # Try to acquire the target if we do not have one yet.
            if self.tracked_id == -1:
                self.tracked_id = self.find_target_by_direction(
                    self.target_direction
                )

                if self.tracked_id != -1:
                    self.tracking_state = "TRACKING"

            # Get the current target state.
            with self.tracker_lock:
                target = self.current_tracks.get(self.tracked_id)

            if target is not None:

                feedback_msg.tracked_id = int(self.tracked_id)
                feedback_msg.x = float(target["x"])
                feedback_msg.y = float(target["y"])
                feedback_msg.state = self.tracking_state

            else:

                feedback_msg.tracked_id = int(self.tracked_id)
                feedback_msg.x = 0.0
                feedback_msg.y = 0.0
                feedback_msg.state = self.tracking_state

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)

        goal_handle.succeed()

        result = TrackPerson.Result()
        result.success = True
        result.tracked_id = int(self.tracked_id)

        return result


    def scan_callback(self, msg):

        current_tracks, img_color = self.lidar_processor.process_scan(msg)

        with self.tracker_lock:
            self.current_tracks = current_tracks.copy()

        # VISUALIZATION AND TF2 BROADCASTING

        for obj_id, track in current_tracks.items():

            x1, y1, x2, y2 = track["bbox"]

            cx = int((x1 + x2) / 2.0)
            cy = int((y1 + y2) / 2.0)

            pos_x = track["x"]
            pos_y = track["y"]

            # OpenCV Drawing
            cv2.rectangle(img_color,(int(x1), int(y1)),(int(x2), int(y2)),(0, 255, 255),1)

            cv2.circle(img_color,(cx, cy),3,(0, 255, 255),-1)

            cv2.putText(img_color,f"ID: {obj_id}",(int(x1), int(y1) - 5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 0, 0),1)

            # DYNAMIC FRAME PUBLICATION

            t = TransformStamped()

            t.header.stamp = msg.header.stamp
            t.header.frame_id = msg.header.frame_id
            t.child_frame_id = f"lidar_leg_{obj_id}"

            t.transform.translation.x = float(pos_x)
            t.transform.translation.y = float(pos_y)
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)

        img_display = cv2.resize(img_color,(self.img_size * 2, self.img_size * 2),interpolation=cv2.INTER_NEAREST)
        cv2.imshow('LIDAR Tracking',img_display)
        cv2.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)
    node = LidarTrackingNode()

    #Multithreading
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