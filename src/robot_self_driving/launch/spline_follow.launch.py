from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    encoder_node = Node(
        package="robot_self_driving",
        executable="encoder",
    )

    robot_node = Node(
        package="robot_self_driving",
        executable="spline_test",
    )

    imu_node = Node(
        package="imu",
        executable="imu"
    )

    ld.add_action(encoder_node)
    ld.add_action(imu_node)
    ld.add_action(robot_node)

    return ld