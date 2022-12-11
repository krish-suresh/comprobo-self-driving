import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import Encoder

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_note')
        self.ENCODER_PIN_ONE = 23
        self.ENCODER_PIN_TWO = 24
        self.encoder = Encoder.Encoder(self.ENCODER_PIN_ONE, self.ENCODER_PIN_TWO)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.encoder_pub = self.create_publisher(Int64, "/encoder", 10)

    def run_loop(self):
        self.encoder_pub.publish(Int64(data=self.encoder.read()))
        print(Int64(data=self.encoder.read()))

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    rclpy.shutdown()