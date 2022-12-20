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
    def __init__(self):
        super().__init__("teleop_node")
        self.mvt_sub = self.create_subscription(
            DriveCommand, "mvt_command", self.process_mvt_command, 10
        )
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.robot = Robot(self, use_sim=False)

        # Only arm the ESC if it has just been powered
        # self.robot.drive.arm_esc()
        self.current_mvt_cmd: DriveCommand = None

    def process_mvt_command(self, msg: DriveCommand):
        self.current_mvt_cmd = msg

    def run_loop(self):
        self.robot.update()
        self.get_logger().info(str(np.around(self.robot.drive.state, 2)))
        if self.current_mvt_cmd:
            self.robot.set_steering_angle(self.current_mvt_cmd.steering_angle)
            self.robot.set_drive_velocity(self.current_mvt_cmd.speed)


def main(args=None):
    rclpy.init(args=args)
    node = ReceiveTeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()
