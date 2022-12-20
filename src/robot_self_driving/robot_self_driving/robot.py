from .mpc_controller import MPCController
from .lqr_trajectory_follower import AckermanLQRTrajectoryFollower
from .simulated_ackermann_drive import SimulatedAckermannDrive
from .drive import AckermannDrive
from rclpy.node import Node
from sensor_msgs.msg import Image

class Robot:
    """
    The Robot class is responsible for defining dataflow between the controller
    and the drive class (simulated or physical).
    """

    def __init__(self, ros_node: Node, use_sim=False):
        """
        Constructor that defines the current ros_node, drive interface and controller.
        """
        self.ros_node = ros_node
        self.drive = (
            AckermannDrive(self.ros_node) if not use_sim else SimulatedAckermannDrive()
        )
        self.controller = AckermanLQRTrajectoryFollower(self.drive, ros_node)

    def set_steering_angle(self, theta: float):
        """
        Set the current steering angle, theta (rad), through the drive class.
        """
        self.drive.set_steering_angle(theta)

    def set_drive_velocity(self, vel: float):
        """
        Set the current drive velocity, vel (m/s), through the drive class.
        """
        self.drive.set_drive_velocity(vel)

    def update(self):
        """
        Update the current state of the controller and the drive class.
        """
        self.controller.update()
        self.drive.update()
