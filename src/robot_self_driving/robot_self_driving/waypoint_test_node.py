import rclpy
from rclpy.node import Node
from drive_commands.msg import DriveCommand
#from .drive import AckermannDrive
from .robot import Robot
from .geometry import AckermannState
import numpy as np

class WaypointTestNode(Node):
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
        self.robot.controller.follow_waypoints(waypoints)

    def run_loop(self):
        self.robot.update()
        if self.robot.controller.is_following:
            print(f"Current Goal: {self.robot.controller.current_goal}")
            # print(self.robot.drive.get_state())
        else:
            print("Path following ended.")
        

def main(args=None):
    rclpy.init(args=args)
    node = WaypointTestNode()
    rclpy.spin(node)
    rclpy.shutdown()