import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
import serial

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_note')
        self.ENCODER_PORT = '/dev/ttyACM1'
        self.ENCODER_BAUD_RATE = 9600
        self.ENCODER_TIMEOUT = 1
        self.encoder_serial = serial.Serial(self.ENCODER_PORT, self.ENCODER_BAUD_RATE, timeout=self.ENCODER_TIMEOUT)
        self.timer_period = 0.005
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.encoder_pub = self.create_publisher(Int64MultiArray, "/encoder", 10)
        self.curr_vert_encoder_ticks = None
        self.curr_hor_encoder_ticks = None

    def run_loop(self):
        serial_input = self.encoder_serial.readline()
        try:
            serial_input_ints = serial_input.decode().strip().split(',')
            self.curr_vert_encoder_ticks = int(serial_input_ints[0])
            self.curr_hor_encoder_ticks = int(serial_input_ints[1])
        except (UnicodeDecodeError, ValueError, IndexError):
            return
        self.encoder_pub.publish(Int64MultiArray(data=[self.curr_vert_encoder_ticks, self.curr_hor_encoder_ticks]))

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    rclpy.shutdown()