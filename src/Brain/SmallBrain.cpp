#include "include/SmallBrain.hpp"
#include "ros/console.h"

using UInt8 = std_msgs::UInt8;

SmallBrain::SmallBrain(std::shared_ptr<Pose> &posePtr, std::shared_ptr<Goal> &goalPtr) : BigBrainIfc(posePtr, goalPtr)
{}

UInt8 SmallBrain::think()
{
  std_msgs::UInt8 direction;

  if(!isGoalReachable())
  {
    // return another enum that handle "goal is not reachable" scenario
  }

  if(pose->y == goal->y && pose->x == goal->x){
      // TODO! create special command (add it to enum) that will handle "reach goal" scenario
      // currently let rover spinning arround xD
      direction.data = MoveCmd::TURN_RIGHT;
      return direction;
  }

  if(pose->y == goal->y)
  {
    if(goal->x > pose->x)
    {
      if(pose->orientation == autonomy_simulator::RoverPose::ORIENTATION_NORTH)
      {
        direction.data = MoveCmd::TURN_RIGHT;
        return direction;
      }
      else if(pose->orientation == autonomy_simulator::RoverPose::ORIENTATION_SOUTH)
      {
        direction.data = MoveCmd::TURN_RIGHT;
        return direction;
      }
    }
    else if(goal->x < pose->x)
    {
      if(pose->orientation == autonomy_simulator::RoverPose::ORIENTATION_NORTH)
      {
        direction.data = MoveCmd::TURN_LEFT;
        return direction;
      }
      else if(pose->orientation == autonomy_simulator::RoverPose::ORIENTATION_SOUTH)
      {
        direction.data = MoveCmd::TURN_RIGHT;
        return direction;
      }
    }
  }

  if(pose->x == goal->x)
  {
    if(goal->y > pose->y)
    {
      if(pose->orientation == autonomy_simulator::RoverPose::ORIENTATION_EAST)
      {
        direction.data = MoveCmd::TURN_LEFT;
        return direction;
      }
      else if(pose->orientation == autonomy_simulator::RoverPose::ORIENTATION_WEST)
      {
        direction.data = MoveCmd::TURN_RIGHT;
        return direction;
      }
    }
    else if(goal->y < pose->y)
    {
      if(pose->orientation == autonomy_simulator::RoverPose::ORIENTATION_EAST)
      {
        direction.data = MoveCmd::TURN_RIGHT;
        return direction;
      }
      else if(pose->orientation == autonomy_simulator::RoverPose::ORIENTATION_WEST)
      {
        direction.data = MoveCmd::TURN_LEFT;
        return direction;
      }
    }

  }


  if(pose->y > goal->y)
  {
    switch (pose->orientation) {
      case autonomy_simulator::RoverPose::ORIENTATION_NORTH:
          direction.data = MoveCmd::GO_BACKWARD;
          return direction;
        break;
      case autonomy_simulator::RoverPose::ORIENTATION_SOUTH:
          direction.data = MoveCmd::GO_FORWARD;
          return direction;
        break;
    }
  }
  else if(pose->y < goal->y)
  {
    switch (pose->orientation) {
      case autonomy_simulator::RoverPose::ORIENTATION_NORTH:
          direction.data = MoveCmd::GO_FORWARD;
          return direction;
        break;
      case autonomy_simulator::RoverPose::ORIENTATION_SOUTH:
          return direction;
          direction.data = MoveCmd::GO_BACKWARD;
        break;
  }

  if(pose->x > goal->x)
    switch (pose->orientation) {
      case autonomy_simulator::RoverPose::ORIENTATION_WEST:
            direction.data = MoveCmd::GO_FORWARD;
            return direction;
        break;
      case autonomy_simulator::RoverPose::ORIENTATION_EAST:
            direction.data = MoveCmd::GO_BACKWARD;
            return direction;
        break;
    }
  }
  else if(pose->x > goal->x){
    switch (pose->orientation) {
      case autonomy_simulator::RoverPose::ORIENTATION_WEST:
            direction.data = MoveCmd::GO_BACKWARD;
            return direction;
        break;
      case autonomy_simulator::RoverPose::ORIENTATION_EAST:
            direction.data = MoveCmd::GO_FORWARD;
            return direction;
        break;
    }
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
