#include "PathSolver.hpp"
#include "autonomy_simulator/RoverPose.h"
#include "ros/console.h"
#include "ros/init.h"
#include "std_msgs/UInt8.h"
#include <memory>
#include <mutex>

namespace ps{

PathSolver::PathSolver() : poseMutex()
{
  movePublisher = nh.advertise<std_msgs::UInt8>("/rover/move", 10);
  poseSubscriber =
      nh.subscribe("/rover/pose", 1, &PathSolver::poseCallback, this);
  goalSubscriber =
      nh.subscribe("/set_goal", 1, &PathSolver::goalCallback, this);
  pose = std::make_shared<autonomy_simulator::RoverPose>();
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
  const std::lock_guard<std::mutex> lock(poseMutex);
  auto best_move = brain->getBestMove();
  return best_move;
}

bool PathSolver::isGoalReached()
{
  const std::lock_guard<std::mutex> lock(poseMutex);
  return pose.value()->x == goal.value()->x && pose.value()->y == goal.value()->y;
}

void PathSolver::poseCallback(const autonomy_simulator::RoverPose::ConstPtr &pose_msg)
{
  const std::lock_guard<std::mutex> lock(poseMutex);
  if(!pose.has_value())
    pose = std::make_shared<autonomy_simulator::RoverPose>(*pose_msg);
  else
  {
    pose.value()->x = pose_msg->x;
    pose.value()->y = pose_msg->y;
    pose.value()->orientation = pose_msg->orientation;
  }
}

void PathSolver::goalCallback(const autonomy_simulator::SetGoal::ConstPtr &goal_msg)
{
  if (!goal.has_value())
  {
    ROS_INFO("Received goal: %d, %d", goal_msg->x, goal_msg->y);
    goal = std::make_shared<autonomy_simulator::SetGoal>(*goal_msg);
    brain = SmallBrain(pose.value(), goal.value());
  }
  else {
    ROS_INFO("Received goal: %d, %d", goal_msg->x, goal_msg->y);
    goal.value()->x = goal_msg->x; 
    goal.value()->y = goal_msg->y; 
  }
}

void PathSolver::sendMove(const ros::TimerEvent &event)
{
  if (!isGoalReached())
  {
      auto direction = solvePath();
      if(direction.data == MoveCmd::GOAL_REACHED)
      {
          ROS_INFO("Rover reached the goal");
          return;
      }
      movePublisher.publish(direction);
    // moveTimer.stop(); // actually I leave it commented bc it works pretty well when goal changes
  }
  else {
    ROS_INFO("Rover reached the goal");
    return;
  }
}
}

