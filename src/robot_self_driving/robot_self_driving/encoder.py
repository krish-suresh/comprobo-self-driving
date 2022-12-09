import rclpy
from rclpy.node import Node
import Encoder
from std_msgs.msg import Float64
import numpy as np

ENCODER_GPIO_PIN_ONE = 14
ENCODER_GPIO_PIN_TWO = 15
TOTAL_ENCODER_TICKS = 8192
WHEEL_RADIUS = .03

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.imu_sub = self.create_subscription(Float64, "/imu_yaw", self.process_imu, 10)
        self.curr_heading = None

        self.encoder = Encoder.Encoder(ENCODER_GPIO_PIN_ONE, ENCODER_GPIO_PIN_TWO)
        self.curr_ticks = self.encoder.read()
        self.prev_ticks = self.curr_ticks

        self.pos_x = 0
        self.pos_y = 0

        self.timer = self.create_timer(.1, self.run_loop)

    def run_loop(self):
        self.update_odom()

    def update_odom(self):
        self.curr_ticks = self.encoder.read()
        delta_ticks = self.curr_ticks - self.prev_ticks
        self.prev_ticks = self.curr_ticks
        dist_travelled = delta_ticks/TOTAL_ENCODER_TICKS*(2*np.pi*WHEEL_RADIUS)
        print(self.curr_heading)
        if self.curr_heading is not None:
            self.pos_x += dist_travelled * np.cos(self.curr_heading)
            self.pos_y += dist_travelled * np.sin(self.curr_heading)
        print(f"X: {self.pos_x}, Y: {self.pos_y}")

    def process_imu(self, msg: Float64):
        self.curr_heading = np.deg2rad(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()
