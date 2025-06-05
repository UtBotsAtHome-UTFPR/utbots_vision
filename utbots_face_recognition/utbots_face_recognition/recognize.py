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
from utbots_actions.action import NewFace, Recognition, Train
from utbots_face_recognition.modules.new_face import PictureTaker
from rclpy.callback_groups import ReentrantCallbackGroup


class RecognizeAction(Node):

    def __init__(self):
        super().__init__('Recognition')

        self.bridge = CvBridge()

        self.new_face = ActionServer(
            self,
            NewFace,
            'new_face',
            self.new_face_cb)
        
        self.new_face = PictureTaker()

    def new_face_cb(self, goal_handle):
        self.get_logger().info('Executing New Face goal...')

        goal = goal_handle.request
        
        n_pics = goal.n_pictures
        name = goal.name + "/"

        path = self.new_face.picture_path_maker(name)
        try:
            for i in range(len(n_pics)):
                
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal preempted (canceled).')
                    goal_handle.canceled()
                    return NewFace.Result()

                # Ler imagem do tópico do usb_cam e converter para cv_img com cv_bridge
                img = wait_for_message(
                    Image,
                    self,
                    '/image_raw',
                    timeout=2,
                    callback_group=ReentrantCallbackGroup()
                )

                cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')

                img = self.new_face.crop_img(cv_image)
                self.new_face.save_img(path + name + str(i), img)

        except TimeoutError:

            self.get_logger().error('Timeout waiting for image.')
            goal_handle.abort()
            return NewFace.Result()

        result = NewFace.Result()
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()