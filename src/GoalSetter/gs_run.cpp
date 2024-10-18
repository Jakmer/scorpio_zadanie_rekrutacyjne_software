#include <ros/ros.h>
#include "autonomy_simulator/RoverPose.h"
#include <iostream>
#include <std_msgs/UInt8.h>
#include "GoalSetter.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_setter");

    GoalSetter gs{};
    gs.run();

    return 0;
}
