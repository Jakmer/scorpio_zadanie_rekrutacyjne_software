#include "include/BigBrain.hpp" 

using UInt8 = std_msgs::UInt8;

BigBrain::BigBrain(std::shared_ptr<Pose> &posePtr, std::shared_ptr<Goal> &goalPtr) : BigBrainIfc(posePtr, goalPtr)
{

}

UInt8 BigBrain::think()
{
    return UInt8();
}

bool BigBrain::validateMove(UInt8 &move)
{
  return true;
}

bool BigBrain::isGoalReachable()
{
  return true;
}

