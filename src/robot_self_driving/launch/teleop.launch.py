"""
ROS launch file for the teleop behavior running on the robot. This file runs
the IMU node, odometry node for encoder data, and the receive_teleop node to
receive drive commands from an external source.
"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    encoder_node = Node(
        package="robot_self_driving",
        executable="odometry",
    )

    robot_node = Node(
        package="robot_self_driving",
        executable="receive_teleop",
    )

    imu_node = Node(
        package="imu",
        executable="imu"
    )

    ld.add_action(encoder_node)
    ld.add_action(imu_node)
    ld.add_action(robot_node)

    return ld