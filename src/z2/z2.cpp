#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "z2");
    ros::NodeHandle nh;

    ROS_INFO("Hello, this is a z2 node!");
    ROS_WARN("Hello, this is a z2 node!");
    ROS_ERROR("Hello, this is a z2 node!");


    return 0;
}
