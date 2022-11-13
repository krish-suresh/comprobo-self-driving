import rclpy
from rclpy.node import node
from drive_commands.msg import DriveCommand
from robot import Robot

class ReceiveTeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.mvt_sub = self.create_subscriber(DriveCommand, "mvt_command", self.process_mvt_command, 10)
        self.robot = Robot()

    def process_mvt_command(self, msg: DriveCommand):
        Robot.set_steering_angle(msg.steering_angle)
        Robot.set_drive_speed(msg.speed)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()