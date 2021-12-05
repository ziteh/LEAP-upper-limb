from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
             IncludeLaunchDescription(
                 PythonLaunchDescriptionSource(
                     [ThisLaunchFileDir(),
                     "/simple_3r_arm.launch.py"]
                 )
             ),

             Node(
                 package="simple_3r_arm_control",
                 executable="ik"
             ),
 
             Node(
                 package="simple_3r_arm_control",
                 executable="simple_3r_arm_forward_kinematics_publisher"
             ),
 
             Node(
                 package="simple_3r_arm_control",
                 executable="simple_3r_arm_relative_controller_topic"
             )
        ]
    )
