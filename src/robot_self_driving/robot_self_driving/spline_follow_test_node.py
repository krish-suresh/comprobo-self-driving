import rclpy
from rclpy.node import Node
from drive_commands.msg import DriveCommand
#from .drive import AckermannDrive
from .robot import Robot
from self_driving_utils.self_driving_utils.geometry import AckermannState
from self_driving_utils.self_driving_utils.curves import CubicSpline2D
from self_driving_utils.self_driving_utils.trajectory import CubicSplineTrajectory, TrapezoidalMotionProfile
import numpy as np

class SplineFollowTestNode(Node):
    def __init__(self):
        super().__init__('waypoint_test_node')
        timer_period = 0.05
        through_speed = 0
        waypoints = [
            # AckermannState([1, 0, 0, 0, 0])
            AckermannState([1, -0.4, -np.pi/4, 0, through_speed]), 
            AckermannState([1.5, -0.7, 0, 0, through_speed]),
            AckermannState([2, 0, np.pi/2, 0, through_speed]), 
            AckermannState([0.5, 0.4, np.pi, 0, 0])
            ]

        self.timer = self.create_timer(timer_period, self.run_loop)
        self.robot = Robot(use_sim=False)
        # self.robot.drive.arm_esc() # TODO figure out way to detect if already on and only arm if power is off
        xp = [0, 0.5, 1, 0]
        yp = [0, 0, 1, 1.5]
        sp = CubicSpline2D(xp, yp)
        mp = TrapezoidalMotionProfile(sp.s[-1],2,1)        
        trajectory = CubicSplineTrajectory(sp, mp)
        self.robot.controller.follow_trajectory(trajectory)

    def run_loop(self):
        self.robot.update()
        if self.robot.controller.is_following:
            # print(f"Current Goal: {self.robot.controller.current_goal}")
            print(self.robot.drive.get_state())
        else:
            print("Path following ended.")
        

def main(args=None):
    rclpy.init(args=args)
    node = SplineFollowTestNode()
    rclpy.spin(node)
    rclpy.shutdown()