import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseWithCovariance, Pose, Point
import serial
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


class OdometryNode(Node):
    """
    ROS Node that continuously publishes an Odometry object that represents the
    current state of the robot.
    """
    def __init__(self):
        """
        Constructor for OdometryNode. Estabishes serial connection with the arduino
        for odometry data, subscribes to the IMU for orientation, and publishes
        the complete odometry data.
        """
        super().__init__("odometry_node")

        # Establish serial connection with the arduino for encoder data
        self.ENCODER_PORT = "/dev/ttyACM0"
        self.ENCODER_BAUD_RATE = 9600
        self.ENCODER_TIMEOUT = 1
        self.encoder_serial = serial.Serial(
            self.ENCODER_PORT, self.ENCODER_BAUD_RATE, timeout=self.ENCODER_TIMEOUT
        )
        self.curr_encoder_ticks = None
        self.prev_encoder_ticks = np.array([])
        self.start_encoder_ticks = None
        self.TOTAL_ENCODER_TICKS: int = 8192
        self.WHEEL_ENCODER_RADIUS: float = 0.03  # m

        # Subscribe to /imu for orientation data
        self.imu_sub = self.create_subscription(Imu, "/imu", self.process_imu, 10)
        self.start_heading = None
        self.curr_heading = None

        # Create puvlisher to continuously publish Odometry Data
        self.odom_pub = self.create_publisher(Odometry, "/wheeled_odom", 10)
        self.curr_odom = None
        self.x = 0
        self.y = 0

        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.run_loop)

    def process_imu(self, msg: Imu):
        """
        Callback for IMU subscription. Continuously processes IMU data for
        orientation and heading.
        """
        # Convert received orientation (quaternion) for the yaw
        (_, _, yaw) = euler_from_quaternion(
            [
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
                self.orientation.w,
            ]
        )
        if not self.start_heading:
            self.start_heading = yaw
        self.curr_heading = (yaw - self.start_heading + np.pi) % (2 * np.pi) - np.pi

    def run_loop(self):
        """
        Timer callback that is continuously called to publish Odometry data
        """
        # Receive and Decode serial data (try-except to avoid decode errors from data loss)
        serial_input = self.encoder_serial.readline()
        try:
            serial_input_ints = np.array(
                [int(enc_num) for enc_num in serial_input.decode().strip().split(",")]
            )
        except (UnicodeDecodeError, ValueError, IndexError):
            return
        
        if len(self.prev_encoder_ticks) == 0:
            self.start_encoder_ticks = serial_input_ints
            self.prev_encoder_ticks = serial_input_ints - self.start_encoder_ticks
        if not self.curr_heading:
            return
        
        # Determine the change in encoder ticks between time stamps
        self.curr_encoder_ticks = serial_input_ints - self.start_encoder_ticks
        delta_ticks = self.curr_encoder_ticks - self.prev_encoder_ticks
        self.prev_encoder_ticks = self.curr_encoder_ticks

        # Calculate the distance travelled based on the change in encoder ticks
        dist_travelled = (
            delta_ticks
            / self.TOTAL_ENCODER_TICKS
            * (2 * np.pi * self.WHEEL_ENCODER_RADIUS)
        )
        self.x += dist_travelled[0] * np.cos(self.curr_heading) + dist_travelled[
            1
        ] * np.sin(self.curr_heading)
        self.y += dist_travelled[0] * np.sin(self.curr_heading) + dist_travelled[
            1
        ] * np.cos(self.curr_heading)

        # Store the current yaw as a quaternion
        orientation_q_list = quaternion_from_euler(0, 0, self.curr_heading)
        orientation_q = Quaternion(
            x=orientation_q_list[0],
            y=orientation_q_list[1],
            z=orientation_q_list[2],
            w=orientation_q_list[3],
        )

        # Publish the current odometry
        self.curr_odom = Odometry(
            pose=PoseWithCovariance(
                pose=Pose(position=Point(x=self.x, y=self.y), orientation=orientation_q)
            )
        )
        self.odom_pub.publish(self.curr_odom)


def main(args=None):
    """
    Main function to run odometry node.
    """
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()
