#include "PathSolver.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "std_msgs/UInt8.h"
#include <mutex>

PathSolver::PathSolver() : pose(), poseMutex()
{
  movePublisher = nh.advertise<std_msgs::UInt8>("/rover/move", 10);
  poseSubscriber =
      nh.subscribe("/rover/pose", 1, &PathSolver::poseCallback, this);
  goalSubscriber =
      nh.subscribe("/set_goal", 1, &PathSolver::goalCallback, this);
  ROS_INFO("PathSolver initialized.");
}

PathSolver::~PathSolver()
{
  ROS_INFO("PathSolver terminated.");
}

void PathSolver::run()
{
  // Wait for the first goal to be received
  ros::Rate loop_rate(10);

  while (!goal.has_value() && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  moveTimer = nh.createTimer(ros::Duration(0.1), &PathSolver::sendMove, this);
}

std_msgs::UInt8 PathSolver::solvePath()
{
  std_msgs::UInt8 direction;

  const std::lock_guard<std::mutex> lock(poseMutex);
  if (pose.y < goal->y)
  {
    direction.data = 2;
    return direction;
  }
  if (pose.x < goal->x)
  {
    if (pose.orientation == pose.ORIENTATION_NORTH)
    {
      direction.data = 1;
      return direction;
    }
    direction.data = 2;
    return direction;
  }
  return direction;
}

bool PathSolver::isGoalReached()
{
  const std::lock_guard<std::mutex> lock(poseMutex);
  return pose.x == goal->x && pose.y == goal->y;
}

void PathSolver::poseCallback(const autonomy_simulator::RoverPose::ConstPtr &pose_msg)
{
  const std::lock_guard<std::mutex> lock(poseMutex);
  pose = *pose_msg;
}

void PathSolver::goalCallback(const autonomy_simulator::SetGoal::ConstPtr &goal_msg)
{
  if (!goal.has_value())
  {
    ROS_INFO("Received goal: %d, %d", goal_msg->x, goal_msg->y);
    goal = *goal_msg;
  }
}

void PathSolver::sendMove(const ros::TimerEvent &event)
{
  if (isGoalReached())
  {
    return;
    // moveTimer.stop(); // actually I leave it commented bc it works pretty well when goal changes
  }
  auto direction = solvePath();
  movePublisher.publish(direction);
}
