import rclpy
from rclpy.node import Node
from drive_commands.msg import DriveCommand
import numpy as np
from robot_self_driving.robot import Robot 
from robot_self_driving.geometry import AckermannState
from robot_self_driving.curves import CubicSpline2D, RaceTrack
from robot_self_driving.trajectory import CubicSplineTrajectory, TrapezoidalMotionProfile, RotationLimitedMotionProfile


class Visualizer(Node):
    """
    """

    def __init__(self):
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.robot = Robot(self, use_sim=False)


    def run_looop(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    rclpy.spin(node)
    rclpy.shutdown()
