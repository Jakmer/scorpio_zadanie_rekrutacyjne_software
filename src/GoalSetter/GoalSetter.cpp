#include "GoalSetter.hpp"

GoalSetter::GoalSetter()
{
  setGoalPublisher =
      nh.advertise<autonomy_simulator::SetGoal>("/set_goal", 100);
  setGoalSubscriber =
      nh.subscribe("/set_goal", 1, &GoalSetter::setGoalCallback, this);
  goal.x = 0;
  goal.y = 0;
  ROS_INFO("GoalSetter initialized.");
}

GoalSetter::~GoalSetter() { ROS_INFO("GoalSetter terminated"); }

void GoalSetter::setGoalCallback(
    const autonomy_simulator::SetGoal::ConstPtr &goal_msg)
{
  this->goal = *goal_msg;
}

void GoalSetter::run()
{
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    setGoalPublisher.publish(goal);
    ROS_INFO("Published goal: (%d, %d)", goal.x, goal.y);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
