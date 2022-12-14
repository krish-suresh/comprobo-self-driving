import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseWithCovariance, Pose, Point
import serial
from tf_transformations import euler_from_quaternion
import numpy as np

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.ENCODER_PORT = '/dev/ttyACM1'
        self.ENCODER_BAUD_RATE = 9600
        self.ENCODER_TIMEOUT = 1
        self.encoder_serial = serial.Serial(self.ENCODER_PORT, self.ENCODER_BAUD_RATE, timeout=self.ENCODER_TIMEOUT)
        self.imu_sub = self.create_subscription(Imu, self.process_imu, 10)
        self.start_heading = None
        self.curr_heading = None
        self.orientation: Quaternion = None
        self.odom_pub = self.create_publisher(Odometry, '/wheeled_odom', 10)
        self.curr_odom = None
        self.timer_period = 0.005
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.curr_encoder_ticks = None
        self.prev_encoder_ticks = None
        self.start_encoder_ticks = None
        self.TOTAL_ENCODER_TICKS: int = 8192
        self.WHEEL_ENCODER_RADIUS: float = .03 # m
        self.x = 0
        self.y = 0

    def process_imu(self, msg: Imu):
        self.orientation = msg.orientation
        self.angular_vel = msg.angular_velocity
        (_, _, yaw) = -euler_from_quaternion([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        if not self.start_heading:
            self.start_heading = yaw
        self.curr_heading = (yaw - self.start_heading + np.pi) % (2 * np.pi) - np.pi

    def run_loop(self):
        serial_input = self.encoder_serial.readline()
        try:
            serial_input_ints = np.array(serial_input.decode().strip().split(','))
        except (UnicodeDecodeError, ValueError, IndexError):
            return
        if not self.prev_encoder_ticks:
            self.prev_encoder_ticks = serial_input_ints
            self.start_encoder_ticks = self.prev_encoder_ticks
        self.curr_encoder_ticks = serial_input_ints - self.start_encoder_ticks
        delta_ticks = self.curr_encoder_ticks - self.prev_encoder_ticks
        dist_travelled = delta_ticks/self.TOTAL_ENCODER_TICKS*(2*np.pi*self.WHEEL_ENCODER_RADIUS)
        self.x += dist_travelled[0] * np.cos(self.curr_heading) + dist_travelled[1] * np.sin(self.curr_heading)
        self.y += dist_travelled[0] * np.sin(self.curr_heading) + dist_travelled[1] * np.cos(self.curr_heading)
        
        self.curr_odom = Odometry(
            pose = PoseWithCovariance(
                pose = Pose(
                    position = Point(x=self.x, y=self.y),
                    orientation = self.orientation
                )
            )
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()