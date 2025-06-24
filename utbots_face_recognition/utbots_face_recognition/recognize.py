import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from sensor_msgs.msg import Image, RegionOfInterest
from rclpy.task import Future
from cv_bridge import CvBridge
from rclpy import wait_for_message

from utbots_actions.action import NewFace, Recognition#, Train
from utbots_msgs.msg import BoundingBox, BoundingBoxes

from utbots_face_recognition.modules.new_face import PictureTaker
from utbots_face_recognition.modules.recognize import Recognize_Action
from rclpy.callback_groups import ReentrantCallbackGroup
import cv2

import time

from rclpy.executors import Future
from rclpy.task import Future
from rclpy.qos import qos_profile_sensor_data


class RecognizeAction(Node):

    def __init__(self):
        super().__init__('Recognition')

        self.bridge = CvBridge()

        self.new_face_as = ActionServer(
            self,
            NewFace,
            'new_face',
            self.new_face_cb
        )
        
        self.new_face = PictureTaker()

        self.recognition_as = ActionServer(
            self,
            Recognition,
            'recognition',
            self.recognition_cb
        )

        self.recognition = Recognize_Action()


    def wait_for_image_message(self, topic='/image_raw', timeout=2.0):
        future = Future()

        def callback(msg):
            if not future.done():
                future.set_result(msg)

        sub = self.create_subscription(
            Image,
            topic,
            callback,
            qos_profile_sensor_data
        )

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        self.destroy_subscription(sub)

        if future.done():
            return future.result()
        else:
            raise TimeoutError(f"No message received on {topic} within {timeout} seconds.")
        
    
    def recognition_cb(self, goal_handle):
        self.get_logger().info('Executing Recognition')

        goal = goal_handle.request

        if goal.image.data:
            img = goal.image.data
        else:
            img = self.wait_for_image_message()


        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')

        people = self.recognition.recognize(cv_image)

        if not people:
            self.get_logger().info('Failed, no faces detected')
            goal_handle.abort()
            return Recognition.Result()

        result = Recognition.Result()
        

        for i in range(len(people)):
            bbox = BoundingBox()
            
            #self.get_logger().info(str(people))

            bbox.id = people[i]["identity"]
            bbox.category = "Person"

            bbox.xmax = people[i]["facial_area"]["x"] + people[i]["facial_area"]["w"]
            bbox.xmin = people[i]["facial_area"]["x"]
            bbox.ymax = people[i]["facial_area"]["y"] + people[i]["facial_area"]["h"]
            bbox.ymin = people[i]["facial_area"]["y"]

            result.people.append(bbox)
        
        marked_img = self.recognition.draw_rec_on_faces(cv_image, result.people)

        pub_img = self.bridge.cv2_to_imgmsg(marked_img, encoding='bgr8')

        result.image = pub_img

        goal_handle.succeed()

        self.get_logger().info('Recognition succeeded')
        
        return result
    
    def new_face_cb(self, goal_handle):
        self.get_logger().info('Executing New Face and saving images in:')

        goal = goal_handle.request
        
        n_pics = goal.n_pictures.data
        name = goal.name.data + ""

        path = self.new_face.picture_path_maker(name)
        self.get_logger().info(path)

        try:
            i = 0
            while i < n_pics:

                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal preempted (canceled).')
                    goal_handle.canceled()
                    return NewFace.Result()

                # Ler imagem do tópico do usb_cam e converter para cv_img com cv_bridge
                img = self.wait_for_image_message()

                cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')

                img = self.new_face.crop_img(cv_image)

                if img is None:
                    i -= 1
                    self.get_logger().info("Either no faces or too many faces")
                else:
                    self.get_logger().info(f"{i+1} out of {n_pics} taken")
                    self.new_face.save_img(path + str(i) + ".jpeg", img)
                i += 1
                time.sleep(1)

        except TimeoutError:

            self.get_logger().error('Timeout waiting for image.')
            goal_handle.abort()
            return NewFace.Result()

        result = NewFace.Result()
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    recognition_action_server = RecognizeAction()

    rclpy.spin(recognition_action_server)


if __name__ == '__main__':
    main()