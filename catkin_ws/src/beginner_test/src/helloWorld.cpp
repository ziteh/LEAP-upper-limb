#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hello_world_cpp"); // Init. Set the node name.
    ros::NodeHandle handler;

    while (ros::ok())
    {
        ROS_INFO("Hello World!"); // Print "Hello World!".
        ros::Duration(1).sleep(); // Delay 1 sec.
    }
}