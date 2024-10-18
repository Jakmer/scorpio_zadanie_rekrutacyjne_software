#ifndef PATH_SOLVER_HPP
#define PATH_SOLVER_HPP

#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/SetGoal.h"
#include "ros/timer.h"
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <optional>
#include <mutex>

class PathSolver
{
public:
  PathSolver();
  ~PathSolver();
  void run();

private:
  ros::NodeHandle nh;
  ros::Publisher movePublisher;
  ros::Subscriber poseSubscriber;
  ros::Subscriber goalSubscriber;
  ros::Timer moveTimer;
  autonomy_simulator::RoverPose pose;
  std::optional<autonomy_simulator::SetGoal> goal;
  std::mutex poseMutex;

  std_msgs::UInt8 solvePath();
  void sendMove(const ros::TimerEvent &event);
  bool isGoalReached();
  void poseCallback(const autonomy_simulator::RoverPose::ConstPtr &pose_msg);
  void goalCallback(const autonomy_simulator::SetGoal::ConstPtr &goal_msg);
};

#endif
