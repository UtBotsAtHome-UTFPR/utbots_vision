import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from utbots_msgs.msg import BoundingBoxes
from utbots_actions.action import TrackPerson

from math import radians, degrees



class TrackPersonClient(Node):

    def __init__(self):
        super().__init__('track_person_client')

        # Action client
        self.action_client = ActionClient(
            self,
            TrackPerson,
            'track_person'
        )

        # Bounding box subscription
        self.boxes_sub = self.create_subscription(
            BoundingBoxes,
            '/utbots/vision/detection/bounding_boxes',
            self.boxes_callback,
            10
        )

        self.target_direction = None
        self.goal_sent = False

        self.get_logger().info('Track Person Action Client started.')

    def boxes_callback(self, msg):
        if self.goal_sent:
            return

        if len(msg.bounding_boxes) == 0:
            return

        # ignore Unknown detections and use the first eligible person.
        for person in msg.bounding_boxes:

            self.get_logger().info(
                f'Detected category: {person.category}'
            )

            if person.category == "Unknown":
                continue

            direction = self.estimate_yaw(person)

            self.target_direction = direction

            self.get_logger().info(
                f'Initial direction: {degrees(direction):.2f} degrees '
                f'({direction:.4f} rad)'
            )

            self.send_goal(direction)
            break

    def estimate_yaw(self, person):
        # Reproduces the direction calculation
        width = 1280  # CHANGE TO 1920
        theta_max = 78 / 2  # Logitech cam FOV

        x = int((person.xmin + person.xmax) / 2)

        x = x - int(width / 2.0)

        theta = radians(theta_max * x / (width / 2))

        angle = degrees(theta)

        center_x = int((person.xmin + person.xmax) / 2)

        self.get_logger().info(
            f'BBox center X: {center_x}'
        )

        return radians(-angle)

    def send_goal(self, direction):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'TrackPerson action server is not available yet.'
            )
            return

        goal_msg = TrackPerson.Goal()
        goal_msg.initial_direction = float(direction)

        self.get_logger().info(
            f'Sending TrackPerson goal: '
            f'{goal_msg.initial_direction:.4f} rad'
        )

        self.goal_sent = True

        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('TrackPerson goal was rejected.')
            self.goal_sent = False
            return

        self.get_logger().info('TrackPerson goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'Feedback | '
            f'ID={feedback.tracked_id} '
            f'position=({feedback.x:.2f}, {feedback.y:.2f}) '
            f'state={feedback.state}'
        )

    def result_callback(self, future):
        result = future.result().result

        self.get_logger().info(
            f'TrackPerson finished | '
            f'success={result.success} '
            f'tracked_id={result.tracked_id}'
        )

        self.goal_sent = False


def main(args=None):
    rclpy.init(args=args)

    node = TrackPersonClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
