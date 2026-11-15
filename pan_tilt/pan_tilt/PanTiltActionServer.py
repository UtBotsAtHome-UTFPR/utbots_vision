#!/usr/bin/env python3
import time
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

from pan_tilt.action import PanSweep, TiltSweep, ScanSequence, MoveToAngles

class PanTiltActionServer(Node):
    def __init__(self):
        super().__init__('pan_tilt_action_server')

        # Parameters
        self.declare_parameter('micro_ros_topic', '/servo_angles')
        self.declare_parameter('publish_degrees', False)
        self.declare_parameter('control_rate_hz', 50.0)

        self.micro_ros_topic = self.get_parameter('micro_ros_topic').value
        self.publish_degrees = self.get_parameter('publish_degrees').value
        self.rate_hz = self.get_parameter('control_rate_hz').value
        self.dt = 1.0 / self.rate_hz

        # Current Joint State (radians)
        self.current_pan = 0.0
        self.current_tilt = 0.0

        # Preemption management: only one motion action runs at a time
        self._goal_lock = threading.Lock()
        self._active_goal_handle = None

        # Publishers (Micro-ROS + URDF JointState)
        self.servo_pub = self.create_publisher(Float32MultiArray, self.micro_ros_topic, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Continuous JointState timer for robot_state_publisher (TF tree)
        self.tf_timer = self.create_timer(1.0 / 30.0, self.publish_joint_states)

        # Reentrant group allows concurrent goal callbacks & cancel requests
        cb_group = ReentrantCallbackGroup()

        self._pan_sweep_server = ActionServer(
            self, PanSweep, 'pan_tilt/pan_sweep',
            execute_callback=self.execute_pan_sweep,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )

        self._tilt_sweep_server = ActionServer(
            self, TiltSweep, 'pan_tilt/tilt_sweep',
            execute_callback=self.execute_tilt_sweep,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )

        self._sequence_server = ActionServer(
            self, ScanSequence, 'pan_tilt/scan_sequence',
            execute_callback=self.execute_scan_sequence,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )

        self._move_to_server = ActionServer(
            self, MoveToAngles, 'pan_tilt/move_to_angles',
            execute_callback=self.execute_move_to,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=cb_group
        )

        self.get_logger().info('Pan-Tilt Action Server ready (4 actions active).')

    # --- Hardware & TF Output Helpers ---

    def send_servo_command(self, pan_rad: float, tilt_rad: float):
        """Updates internal state and sends command to micro-ROS."""
        self.current_pan = pan_rad
        self.current_tilt = tilt_rad

        msg = Float32MultiArray()
        if self.publish_degrees:
            p_deg = max(0.0, min(180.0, math.degrees(pan_rad) + 90.0))
            t_deg = max(0.0, min(180.0, math.degrees(tilt_rad) + 90.0))
            msg.data = [p_deg, t_deg]
        else:
            msg.data = [float(pan_rad), float(tilt_rad)]

        self.servo_pub.publish(msg)

    def publish_joint_states(self):
        """Feeds robot_state_publisher to keep the URDF TF tree live."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['pan_joint', 'tilt_joint']
        js.position = [float(self.current_pan), float(self.current_tilt)]
        self.joint_pub.publish(js)

    # --- Preemption & Lifecycle Callbacks ---

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('External preemption/cancellation requested.')
        return CancelResponse.ACCEPT

    def _register_active_goal(self, goal_handle):
        """Aborts any previously running action so the new one takes over cleanly."""
        with self._goal_lock:
            if self._active_goal_handle is not None and self._active_goal_handle.is_active:
                self.get_logger().info('Preempting previous active goal with new goal.')
                self._active_goal_handle.abort()
            self._active_goal_handle = goal_handle

    def _is_interrupted(self, goal_handle) -> bool:
        """Returns True if canceled externally OR aborted by a newer goal."""
        return goal_handle.is_cancel_requested or not goal_handle.is_active

    # --- Motion Primitives ---

    def _move_linear(self, goal_handle, target_pan: float, target_tilt: float, speed: float, feedback_cb=None) -> bool:
        """
        Smoothly interpolates from current position to (target_pan, target_tilt) at `speed` rad/s.
        Returns True if reached, False if interrupted/preempted.
        """
        speed = max(0.05, abs(speed))
        max_step = speed * self.dt

        while rclpy.ok():
            if self._is_interrupted(goal_handle):
                return False

            d_pan = target_pan - self.current_pan
            d_tilt = target_tilt - self.current_tilt
            dist = math.hypot(d_pan, d_tilt)

            if dist <= max_step:
                self.send_servo_command(target_pan, target_tilt)
                if feedback_cb:
                    feedback_cb(0.0)
                return True

            # Step proportionally toward target
            ratio = max_step / dist
            next_pan = self.current_pan + d_pan * ratio
            next_tilt = self.current_tilt + d_tilt * ratio
            self.send_servo_command(next_pan, next_tilt)

            if feedback_cb:
                feedback_cb(dist - max_step)

            time.sleep(self.dt)
        return False

    def _interruptible_sleep(self, goal_handle, duration: float) -> bool:
        """Sleeps in small increments so cancellation is instantaneous."""
        elapsed = 0.0
        while elapsed < duration and rclpy.ok():
            if self._is_interrupted(goal_handle):
                return False
            step = min(self.dt, duration - elapsed)
            time.sleep(step)
            elapsed += step
        return True

    # --- Action 1: Pan Sweep Until Preempted ---

    def execute_pan_sweep(self, goal_handle):
        self._register_active_goal(goal_handle)
        goal = goal_handle.request
        feedback = PanSweep.Feedback()
        result = PanSweep.Result()

        # First move to starting tilt height
        if not self._move_linear(goal_handle, self.current_pan, goal.tilt_angle, goal.speed):
            return self._finalize_sweep(goal_handle, result, 'Preempted during initial tilt alignment')

        target = goal.max_pan
        while rclpy.ok():
            reached = self._move_linear(
                goal_handle, target, goal.tilt_angle, goal.speed,
                feedback_cb=lambda _: self._pub_sweep_fb(goal_handle, feedback)
            )
            if not reached:
                return self._finalize_sweep(goal_handle, result, 'Stopped by external preemption')

            # Flip sweep direction
            target = goal.min_pan if math.isclose(target, goal.max_pan) else goal.max_pan

    # --- Action 3: Tilt Sweep Until Preempted ---

    def execute_tilt_sweep(self, goal_handle):
        self._register_active_goal(goal_handle)
        goal = goal_handle.request
        feedback = TiltSweep.Feedback()
        result = TiltSweep.Result()

        # First align pan angle
        if not self._move_linear(goal_handle, goal.pan_angle, self.current_tilt, goal.speed):
            return self._finalize_sweep(goal_handle, result, 'Preempted during initial pan alignment')

        target = goal.max_tilt
        while rclpy.ok():
            reached = self._move_linear(
                goal_handle, goal.pan_angle, target, goal.speed,
                feedback_cb=lambda _: self._pub_sweep_fb(goal_handle, feedback)
            )
            if not reached:
                return self._finalize_sweep(goal_handle, result, 'Stopped by external preemption')

            # Flip sweep direction
            target = goal.min_tilt if math.isclose(target, goal.max_tilt) else goal.max_tilt

    def _pub_sweep_fb(self, goal_handle, feedback):
        feedback.current_pan = float(self.current_pan)
        feedback.current_tilt = float(self.current_tilt)
        goal_handle.publish_feedback(feedback)

    def _finalize_sweep(self, goal_handle, result, reason: str):
        result.final_pan = float(self.current_pan)
        result.final_tilt = float(self.current_tilt)
        result.stop_reason = reason
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        return result

    # --- Action 2: Choreographed Scan Sequence ---

    def execute_scan_sequence(self, goal_handle):
        self._register_active_goal(goal_handle)
        goal = goal_handle.request
        feedback = ScanSequence.Feedback()
        result = ScanSequence.Result()

        p = abs(goal.pan_amplitude)
        t = abs(goal.tilt_amplitude)
        spd = goal.speed
        pause = max(0.0, goal.pause_duration)

        # Waypoints: (stage_name, target_pan, target_tilt, pause_after)
        waypoints = [
            ('1_move_left',         p,    0.0, True),
            ('2_left_look_up',      p,    t,   False),
            ('3_left_look_down',    p,   -t,   False),
            ('4_left_tilt_center',  p,    0.0, True),
            ('5_center_sideways',   0.0,  0.0, True),
            ('6_center_look_up',    0.0,  t,   False),
            ('7_center_look_down',  0.0, -t,   False),
            ('8_center_tilt_center',0.0,  0.0, True),
            ('9_move_right',       -p,    0.0, True),
            ('10_right_look_up',    -p,    t,   False),
            ('11_right_look_down',  -p,   -t,   False),
            ('12_right_tilt_center',-p,    0.0, True),
            ('13_center_sideways',   0.0,  0.0, True),
        ]

        for stage_name, t_pan, t_tilt, should_pause in waypoints:
            feedback.current_stage = stage_name

            def on_step(_):
                feedback.current_pan = float(self.current_pan)
                feedback.current_tilt = float(self.current_tilt)
                goal_handle.publish_feedback(feedback)

            if not self._move_linear(goal_handle, t_pan, t_tilt, spd, feedback_cb=on_step):
                result.completed = False
                result.last_stage = f'Preempted during {stage_name}'
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                return result

            if should_pause and pause > 0.0:
                feedback.current_stage = f'{stage_name}_stopped'
                goal_handle.publish_feedback(feedback)
                if not self._interruptible_sleep(goal_handle, pause):
                    result.completed = False
                    result.last_stage = f'Preempted while stopped at {stage_name}'
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    return result

        goal_handle.succeed()
        result.completed = True
        result.last_stage = 'completed_all_stages'
        return result

    # --- Action 4: Move to Set Angles ---

    def execute_move_to(self, goal_handle):
        self._register_active_goal(goal_handle)
        goal = goal_handle.request
        feedback = MoveToAngles.Feedback()
        result = MoveToAngles.Result()

        def on_step(rem_dist):
            feedback.distance_remaining = float(rem_dist)
            goal_handle.publish_feedback(feedback)

        reached = self._move_linear(
            goal_handle, goal.target_pan, goal.target_tilt, goal.speed, feedback_cb=on_step
        )

        result.final_pan = float(self.current_pan)
        result.final_tilt = float(self.current_tilt)

        if reached:
            result.success = True
            goal_handle.succeed()
        else:
            result.success = False
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
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
