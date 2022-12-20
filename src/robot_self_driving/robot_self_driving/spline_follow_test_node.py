import rclpy
from rclpy.node import Node
from .robot import Robot
from .curves import CubicSpline2D, RaceTrack
from .trajectory import (
    CubicSplineTrajectory,
    RotationLimitedMotionProfile,
)

class SplineFollowTestNode(Node):
    """
    ROS Node to manage following a pre-determined spline
    """
    def __init__(self):
        super().__init__("waypoint_test_node")
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.robot = Robot(self, use_sim=True)
        # self.robot.drive.arm_esc() # TODO figure out way to detect if already on and only arm if power is off
        # track = RaceTrack("/home/ubuntu/ros_ws/src/comprobo_self_driving/src/robot_self_driving/tracks/Monza_raceline.csv", 35, 9)
        track = RaceTrack(
            "/home/ubuntu/ros_ws/src/comprobo_self_driving/src/robot_self_driving/tracks/Budapest_raceline.csv",
            20,
            9,
        )
        # Based on the points in the race track, construct a cubic spline trajectory to follow
        x, y = track.access_coords()
        sp = CubicSpline2D(x, y)
        mp = RotationLimitedMotionProfile(sp, 1, 0.5, 0.5, 0.01)
        trajectory = CubicSplineTrajectory(sp, mp)

        # Follow the calculated trajectory based on a LQR controller
        self.robot.controller.follow_trajectory(trajectory)
        self.robot.drive.draw_traj(sp)

    def run_loop(self):
        """
        Callback for the timer to determine if the robot has finished following.
        """
        # Update the current state of the robot
        self.robot.update()
        if self.robot.controller.is_following:
            pass
        else:
            print("Path following ended.")


def main(args=None):
    """
    Main function for the spline following node.
    """
    rclpy.init(args=args)
    node = SplineFollowTestNode()
    rclpy.spin(node)
    rclpy.shutdown()
