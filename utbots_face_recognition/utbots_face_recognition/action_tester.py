import rclpy

from cv_bridge import CvBridge
from utbots_actions.action import NewFace, Recognition#, Train

from rclpy.action import ActionClient
from rclpy.node import Node

class RecognizeActionClient(Node):

    def __init__(self):
        super().__init__('recognize_action_client')
        self.new_face_action_client = ActionClient(self, NewFace, 'new_face')
        self.recognition_action_client = ActionClient(self, Recognition, 'recognition')

    def send_new_face_goal(self):
        goal_msg = NewFace.Goal()
        goal_msg.n_pictures.data = 2
        goal_msg.name.data = "Teste"

        self.new_face_action_client.wait_for_server()

        return self.new_face_action_client.send_goal(goal_msg)
    
    def send_recognition_goal(self):
        goal_msg = Recognition.Goal()

        self.recognition_action_client.wait_for_server()

        return self.recognition_action_client.send_goal(goal_msg)




def main(args=None):
    rclpy.init(args=args)

    action_client = RecognizeActionClient()

    future = action_client.send_new_face_goal()
    rclpy.spin_until_future_complete(action_client, future)

if __name__ == '__main__':
    main()