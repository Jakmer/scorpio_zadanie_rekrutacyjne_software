#include <ros/ros.h>
#include "autonomy_simulator/RoverPose.h"
#include <iostream>

void goalCallback(const autonomy_simulator::RoverPose::ConstPtr& msg) {
    ROS_INFO("Goal received: x=%d, y=%d, orientation=%d", msg->x, msg->y, msg->orientation);
}

void poseCallback(const autonomy_simulator::RoverPose::ConstPtr& msg) {
    ROS_INFO("Current pose: x=%d, y=%d, orientation=%d", msg->x, msg->y, msg->orientation);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "reader");
    ros::NodeHandle nh;

    ROS_INFO("Hello, this is a reader node!");

    ros::Subscriber pose_sub = nh.subscribe("/rover/pose", 10, poseCallback);

    ros::spin();

    return 0;
}
