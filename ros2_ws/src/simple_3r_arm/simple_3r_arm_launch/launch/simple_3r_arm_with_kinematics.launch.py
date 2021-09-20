from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution , ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(),
            "/simple_3r_arm.launch.py"]
        )
    )

    ik_node = Node(
        package="simple_3r_arm_control",
        executable="ik"
    )

    fk_node = Node(
        package="simple_3r_arm_control",
        executable="fk"
    )

    nodes = [ik_node, fk_node]

    return LaunchDescription(
       [base_launch] + nodes 
    )
