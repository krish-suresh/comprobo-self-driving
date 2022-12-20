import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from .odom_fusion_factor_graph import PoseGraphOptimization


class FuseOdometryNode(Node):
    def __init__(self):
        super().__init__('fuse_odometry_node')
        self.vio_sub = self.create_subscription(Odometry, "odom", self.process_vio, 10)
        self.robot_odom_sub = self.create_subscription(Odometry, "robot/odom", self.process_robot_odom, 10)
        self.fused_pub = self.create_publisher(Odometry, "fused_odom", 10)
        self.factor_graph = PoseGraphOptimization(marginalization_buffer_length = 10)


    def process_vio(self, msg: Odometry):
        # integrate pose towards nearest odom time
        # add measurement edge or measurement on the vertex
        pass

    def process_robot_odom(self, robot_odom_msg: Odometry):

        self.factor_graph.add_pose()
        # Add pose
        # add edge to last pose
        # optimize
        # marginalize earliest vertex
        # get pose from latest vertex

        fused_odom_msg : Odometry = Odometry()
        fused_odom_msg.child_frame_id = robot_odom_msg.child_frame_id
        fused_odom_msg.pose = robot_odom_msg.pose
        fused_odom_msg.twist = robot_odom_msg.twist
        fused_odom_msg.header = robot_odom_msg.header
        self.fused_pub.publish(fused_odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FuseOdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()