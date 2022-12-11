import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
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
        self.encoder_pub = self.create_publisher(Int64, "/encoder", 10)

    def run_loop(self):
        serial_input = self.encoder_serial.readline()
        try:
            serial_input_int = int(serial_input.decode().strip())
            print(f'serial: {serial_input_int}')
            self.curr_encoder_ticks = serial_input_int
        except (UnicodeDecodeError, ValueError):
            return
        self.encoder_pub.publish(Int64(data=self.curr_encoder_ticks))

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    rclpy.shutdown()