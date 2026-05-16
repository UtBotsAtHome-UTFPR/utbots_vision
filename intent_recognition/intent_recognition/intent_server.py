import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image 
from utbots_msgs.msg import BoundingBoxes 
from utbots_actions.action import RecognizePointing
from cv_bridge import CvBridge
from ultralytics import YOLO
import mediapipe as mp
from intent_recognition.intent_core import Person, TemporalFilter, TargetBox

class IntentActionServer(Node):
    def __init__(self):
        super().__init__('intent_action_server')
        self.bridge = CvBridge()
        self.action_active = False
        self.final_target = None
        self.current_feedback = "IDLE"
        self.active_engine = None

        # Load weights into VRAM upon node initialization
        self.get_logger().info("Loading Pose models (YOLO and MediaPipe)...")
        self.yolo_pose_engine = YOLO('yolov8n-pose.pt') 
        self.mp_pose = mp.solutions.pose.Pose(min_detection_confidence=0.7)
        
        self.person = Person("Human_1")
        self.filter_logic = TemporalFilter(buffer_size=15, min_votes=8)
        self.dynamic_targets = []
        
        # Declare parameters and subscriptions from the launch file
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, 
            camera_topic, 
            self.image_callback, 
            10
        )
        self.boxes_sub = self.create_subscription(
            BoundingBoxes, 
            '/utbots/vision/detection/bounding_boxes', 
            self.boxes_callback, 
            10
        )
        
#***
        #Just for debbuging on rviz2, if you want to see the image and its segments you can remove this comment
        self.debug_pub = self.create_publisher(Image, '/intent_debug_image', 10)
#***
        # Action Server
        self._action_server = ActionServer(
            self,
            RecognizePointing,
            'recognize_pointing_action',
            self.execute_callback
        )
        self.get_logger().info("Intent Node Online and Waiting for Behavior Tree.")

    def boxes_callback(self, msg):
        # Reads objects processed by the yolo_node and updates targets
        if not self.action_active:
            return
            
        new_targets = []
        for bbox in msg.bounding_boxes:
            target_id = f"{bbox.category}_{bbox.id}" 
            target = TargetBox(target_id, int(bbox.xmin), int(bbox.ymin), int(bbox.xmax), int(bbox.ymax))
            new_targets.append(target)
            
        self.dynamic_targets = new_targets

    def execute_callback(self, goal_handle):
        # Entry point when the Behavior Tree calls the action
        self.active_engine = goal_handle.request.pose_model 
        self.get_logger().info(f"Action started using engine: {self.active_engine}")
        
        # Reset statuses from the last execution
        self.person.state = "SEARCHING"
        self.person.angle_history.clear()
        self.filter_logic.buffer.clear()
        self.final_target = None
        self.action_active = True 
        
        feedback_msg = RecognizePointing.Feedback()

        # Asynchronous loop keeping the BT informed
        while self.final_target is None:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.action_active = False
                self.get_logger().info("Action aborted by BT.")
                return RecognizePointing.Result()

            feedback_msg.current_state = self.current_feedback
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1) 

        # Clean completion
        goal_handle.succeed()
        result = RecognizePointing.Result()
        result.target_bag = self.final_target
        
        self.get_logger().info(f"Success. Operator pointed to: {self.final_target}")
        self.action_active = False 
        return result

    def image_callback(self, msg):
        # Runs inference if the action is active
        if not self.action_active:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        pt_origin, pt_endpoint = None, None

        # Point extraction based on the model passed by parameter
        if self.active_engine == "MEDIAPIPE_POSE":
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.mp_pose.process(img_rgb)
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                elbow = landmarks[mp.solutions.pose.PoseLandmark.RIGHT_ELBOW.value]
                wrist = landmarks[mp.solutions.pose.PoseLandmark.RIGHT_WRIST.value]
                pt_origin = [int(elbow.x * w), int(elbow.y * h), 0]
                pt_endpoint = [int(wrist.x * w), int(wrist.y * h), 0]

        elif self.active_engine == "YOLO_POSE":
            pose_results = self.yolo_pose_engine(frame, verbose=False)
            if pose_results and len(pose_results[0].keypoints.xy) > 0:
                keypoints = pose_results[0].keypoints.xy[0].cpu().numpy()
                if len(keypoints) > 10:
                    elbow, wrist = keypoints[8], keypoints[10]
                    if elbow[0] != 0 and wrist[0] != 0:
                        pt_origin = [int(elbow[0]), int(elbow[1]), 0]
                        pt_endpoint = [int(wrist[0]), int(wrist[1]), 0]

        
        raw_target = None
        if pt_origin and pt_endpoint:
            raw_target = self.person.update_pose(pt_origin, pt_endpoint, self.dynamic_targets)

        confirmed_target = self.filter_logic.update(raw_target)
        self.current_feedback = self.person.state

        if confirmed_target:
            self.final_target = confirmed_target
#***
#If you wish to see the arm segment on rviz2 you can use the following commented part:
        #Draws the arm segment(pulse and elbow)
        if pt_origin and pt_endpoint:
            cv2.line(frame, (pt_origin[0], pt_origin[1]), (pt_endpoint[0], pt_endpoint[1]), (255, 0, 0), 4)
            cv2.circle(frame, (pt_origin[0], pt_origin[1]), 6, (0, 255, 0), -1) # Cotovelo
            cv2.circle(frame, (pt_endpoint[0], pt_endpoint[1]), 6, (0, 0, 255), -1) # Pulso

        #Draws avaliable bounding boxes' center (Yellow)
        for target in self.dynamic_targets:
            cv2.circle(frame, (target.center[0], target.center[1]), 8, (0, 255, 255), -1)
            cv2.putText(frame, target.id, (target.center[0] + 15, target.center[1] + 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
        #If it found the target, it shows on the screen (Pink)
        if self.current_feedback == "LOCKED":
            cv2.putText(frame, f"STATUS: LOCKED | TGT: {confirmed_target}", (30, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200, 0, 200), 3)

        #cv2 to ros msg convertion with cvbridge
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f"Erros on visual debug: {e}")
#***

def main(args=None):
    rclpy.init(args=args)
    node = IntentActionServer()
    # The MultiThreadedExecutor ensures that image_callback and execute_callback run simultaneously
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