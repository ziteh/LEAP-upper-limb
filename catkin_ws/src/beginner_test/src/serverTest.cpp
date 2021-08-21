#include <ros/ros.h>
#include <rospy_tutorials/AddTwoInts.h>

bool add(rospy_tutorials::AddTwoInts::Request &req,
         rospy_tutorials::AddTwoInts::Response &res)
{
    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld",(long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints.");
    ros::spin(); // Keep runing this node.

    return 0;
}