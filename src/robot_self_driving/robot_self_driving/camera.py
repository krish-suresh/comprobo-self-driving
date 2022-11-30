import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

ROS_CAMERA_TOPIC = ""

class CameraReceiverNode(Node):
    def __init__(self):
        super().__init__("camera_receiver_node")
        self.current_image = None
        self.image_size = None
        self.camera_subscription = self.create_subscription(Image, ROS_CAMERA_TOPIC, self.process_image, 10)

    def process_image(self, msg: Image):
        self.current_image = msg.data
        self.image_size = [msg.height, msg.width]


    def get_image_data(self):
        return self.current_image
    
    def get_image_size(self):
        return self.image_size