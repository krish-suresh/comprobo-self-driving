from .mpc_controller import MPCController
from .lqr_trajectory_follower import AckermanLQRTrajectoryFollower
from .simulated_ackermann_drive import SimulatedAckermannDrive
from .drive import AckermannDrive
from rclpy.node import Node
from sensor_msgs.msg import Image

ROS_CAMERA_TOPIC = ''

class Robot():
    """
    """

    def __init__(self, ros_node: Node, use_sim = False):
        """
        """
        self.ros_node = ros_node
        # self.camera_sub = self.ros_node.create_subscription(Image, ROS_CAMERA_TOPIC, self.process_image, 10)
        self.current_image = None
        self.drive = AckermannDrive(self.ros_node) if not use_sim else SimulatedAckermannDrive()
        self.controller = AckermanLQRTrajectoryFollower(self.drive)
        # self.controller = MPCController(self.drive, 0.2) # TODO move tol somewhere else

    def process_image(self, msg: Image):
        self.current_image = msg.data

    def set_steering_angle(self, theta: float):
        """
        """
        self.drive.set_steering_angle(theta)

    def set_drive_velocity(self, vel: float):
        """
        """
        self.drive.set_drive_velocity(vel)
    
    def update(self):
        """
        """
        self.controller.update()
        self.drive.update()