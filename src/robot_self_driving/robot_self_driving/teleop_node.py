import rclpy
from rclpy.node import Node
from drive_commands.msg import DriveCommand
#from .drive import AckermannDrive
from .robot import Robot

class ReceiveTeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.mvt_sub = self.create_subscription(DriveCommand, "mvt_command", self.process_mvt_command, 10)
        self.robot = Robot()
        #self.drive = AckermannDrive()

    def process_mvt_command(self, msg: DriveCommand):
        self.robot.set_steering_angle(msg.steering_angle, 2)
        self.robot.set_drive_velocity(msg.speed, 2)
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReceiveTeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()