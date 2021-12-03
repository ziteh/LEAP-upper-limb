from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    serial_port_node = Node(
        package="serial_port",
        executable="read_write_from_topic_bytemultiarray"
    )

    motor_position_control_sender_node = Node(
        package="data_package",
        executable="motor_position_control_sender"
    )

    # nodes = [serial_port_node, motor_position_control_sender_node]

    return LaunchDescription(
        [serial_port_node, motor_position_control_sender_node]
    )
