#include <ros/ros.h>
#include "autonomy_simulator/RoverPose.h"
#include <iostream>
#include <std_msgs/UInt8.h>
#include "GoalSetter.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "z2");

    GoalSetter gs{};
    gs.run();

    return 0;
}
