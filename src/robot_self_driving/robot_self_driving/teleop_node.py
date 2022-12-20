"""
ROS Node to receive teleoperation instructions from an external
controller
"""

import rclpy
from rclpy.node import Node
from drive_commands.msg import DriveCommand
from .robot import Robot
import numpy as np


class ReceiveTeleopNode(Node):
    """
    ROS node that subscribes to "mvt_command" and sets robot velocities and
    steering angles based on what is received.
    """

    def __init__(self):
        """
        Constructor to create ROS node.
        """
        super().__init__("teleop_node")

        # Subscribe to mvt_command to receive user input
        self.mvt_sub = self.create_subscription(
            DriveCommand, "mvt_command", self.process_mvt_command, 10
        )
        self.current_mvt_cmd: DriveCommand = None

        # Create timer to continuosly send commands to the robot
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.run_loop)

        # Create Robot instance to interface with the hardware
        self.robot = Robot(self, use_sim=False)

        # Only arm the ESC if it has just been powered
        # self.robot.drive.arm_esc()


    def process_mvt_command(self, msg: DriveCommand):
        """
        Callback for mvt_sub. This function updates the current movement
        command being sent to the robot. DriveCommand messages contain
        steering angles and forward velocities.
        """
        self.current_mvt_cmd = msg

    def run_loop(self):
        """
        This is the callback for the timer that is continuously called. This
        is responsible for calling state updates to the robot and, if received,
        send drive commands to the robot.
        """
        self.robot.update()
        self.get_logger().info(str(np.around(self.robot.drive.state, 2)))
        if self.current_mvt_cmd:
            self.robot.set_steering_angle(self.current_mvt_cmd.steering_angle)
            self.robot.set_drive_velocity(self.current_mvt_cmd.speed)


def main(args=None):
    """
    Main function to run teleop node.
    """
    rclpy.init(args=args)
    node = ReceiveTeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()
