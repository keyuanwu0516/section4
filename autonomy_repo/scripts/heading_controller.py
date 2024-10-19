#!/usr/bin/env python3

import numpy
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState


class HeadingController(BaseHeadingController):
    def __init__(self):
        super().__init__()  # Call the parent class's __init__ method

        # Proportional control gain
        self.declare_parameter("kp", 2.0)
        
    @property
    def kp(self) -> float:
        """ Retrieve the real-time value of the kp parameter """
        return self.get_parameter("kp").value


    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        """ Override the base class method to compute control using proportional heading control

        Args:
            state (TurtleBotState): The current state of the robot
            goal (TurtleBotState): The goal state (target pose)

        Returns:
            TurtleBotControl: The control command message
        """
        # Calculate the heading error
        heading_error = wrap_angle(goal.theta - state.theta)

        # Use proportional control formula to compute angular velocity
        omega = self.kp * heading_error

        # Create and return the control message
        control_message = TurtleBotControl()
        control_message.omega = omega
        return control_message


if __name__ == "__main__":
    # Initialize the ROS2 system
    rclpy.init()

    # Create an instance of the HeadingController class
    heading_controller = HeadingController()

    try:
        # Spin the node to keep it running and listening for messages
        rclpy.spin(heading_controller)
    finally:
        # Shutdown the ROS2 system after spinning
        rclpy.shutdown()
