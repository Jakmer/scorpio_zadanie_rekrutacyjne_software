#ifndef GOAL_SETTER_HPP
#define GOAL_SETTER_HPP

#include <ros/ros.h>
#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/SetGoal.h"

class GoalSetter
{
public:
    GoalSetter();
    ~GoalSetter();
    void run();

private:
    ros::NodeHandle nh;
    ros::Publisher setGoalPublisher;
    ros::Subscriber setGoalSubscriber;
    autonomy_simulator::SetGoal goal;

    void setGoalCallback(const autonomy_simulator::SetGoal::ConstPtr &goal);
};

#endif
