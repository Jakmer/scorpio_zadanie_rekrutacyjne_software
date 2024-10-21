#include "include/SmallBrain.hpp"
#include "ros/console.h"

using UInt8 = std_msgs::UInt8;

SmallBrain::SmallBrain(std::shared_ptr<Pose> &posePtr, std::shared_ptr<Goal> &goalPtr) : BigBrainIfc(posePtr, goalPtr)
{}

UInt8 SmallBrain::think()
{
  std_msgs::UInt8 direction;

  if (pose->y < goal->y)
  {
    direction.data = 2;
    return direction;
  }
  if (pose->x < goal->x)
  {
    if (pose->orientation == pose->ORIENTATION_NORTH)
    {
      direction.data = 1;
      return direction;
    }
    direction.data = 2;
    return direction;
  }

    return direction;
}

bool SmallBrain::validateMove(UInt8 &move)
{
    switch (move.data) {
        case 0: 
        case 1: 
            return true;
        case 2: 
            if(pose->orientation == pose->ORIENTATION_EAST && pose->x == 49)
                return false;
            if(pose->orientation == pose->ORIENTATION_WEST && pose->x == 0)
                return false;
            if(pose->orientation == pose->ORIENTATION_NORTH && pose->y == 49)
                return false;
            if(pose->orientation == pose->ORIENTATION_SOUTH && pose->y == 0)
                return false;
            return true;
        case 3: 
            if(pose->orientation == pose->ORIENTATION_EAST && pose->x == 0)
                return false;
            if(pose->orientation == pose->ORIENTATION_WEST && pose->x == 49)
                return false;
            if(pose->orientation == pose->ORIENTATION_NORTH && pose->y == 0)
                return false;
            if(pose->orientation == pose->ORIENTATION_SOUTH && pose->y == 49)
                return false;
            return true;
        default:
            ROS_WARN("Undefined move!");
            return false;
    }
}

bool SmallBrain::isGoalReachable()
{
    return goal->x >= 0 && goal->x < 50 && goal->y > 0 && goal->y < 50;
}
