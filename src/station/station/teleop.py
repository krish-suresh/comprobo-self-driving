import rclpy
from rclpy.node import Node
from drive_commands.msg import DriveCommand
import tty
import select
import sys
import termios

class SendDriveCommand(Node):
    key_to_vel = {
        "s": DriveCommand(
            steering_angle=0.0, speed=0.0
        ),
        "q": DriveCommand(
            steering_angle=0.349, speed=.5
        ),
        "w": DriveCommand(
            steering_angle=0.0, speed=.5
        ),
        "e": DriveCommand(
            steering_angle=-0.349, speed=.5
        ),
        "a": DriveCommand(
            steering_angle=0.349, speed=.5
        ),
        "z": DriveCommand(
            steering_angle=0.349, speed=-.5
        ),
        "x": DriveCommand(
            steering_angle=0.349, speed=-.5
        ),
        "c": DriveCommand(
            steering_angle=-0.349, speed=-.5
        ),
    }

    def __init__(self):
        super().__init__('send_drive_command')
        self.settings = termios.tcgetattr(sys.stdin)
        self.drive_command_pub = self.create_publisher(DriveCommand, "mvt_command", 10)
        self.timer = self.create_timer(.1, self.run_loop)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run_loop(self):
        self.key = self.get_key()
        if self.key == "\x03":
            self.publisher.publish(self.key_to_vel["s"])
            raise KeyboardInterrupt
        if self.key in self.key_to_vel.keys():
            self.drive_command_pub.publish(self.key_to_vel[self.key])

def main(args=None):
    rclpy.init()
    node = SendDriveCommand()
    rclpy.spin(node)
    rclpy.shutdown()