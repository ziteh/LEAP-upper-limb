from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("simple_3r_arm_launch"),
                    "/launch/simple_3r_arm_with_relative_motion_topic.launch.py"]
                )
            ),

            Node(
                package="serial_port",
                executable="read_write_from_topic_bytemultiarray"
            ),

            Node(
                package="data_package",
                executable="force_sensor_value_decoder"
            )
        ]
    )
