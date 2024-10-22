#ifndef BIG_BRAIN_IFC
#define BIG_BRAIN_IFC

#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/SetGoal.h"
#include <std_msgs/UInt8.h>
#include <memory>

using UInt8 = std_msgs::UInt8;
using Pose = autonomy_simulator::RoverPose;
using Goal = autonomy_simulator::SetGoal;

enum MoveCmd{
  TURN_LEFT,
  TURN_RIGHT,
  GO_FORWARD,
  GO_BACKWARD
};

class BigBrainIfc {
public:
    BigBrainIfc(std::shared_ptr<Pose> &posePtr, std::shared_ptr<Goal> &goalPtr)
        : pose(posePtr), goal(goalPtr), prev_move(), fut_move() {}

    virtual ~BigBrainIfc() = default;

    UInt8 getBestMove() { 
        auto bestMove = think();

        if(validateMove(bestMove))
          return bestMove; 
        else 
          // TODO! add "not validated move" handler
          return bestMove;
    }

protected:
    virtual UInt8 think() = 0;
    virtual bool validateMove(UInt8 &move) = 0;
    virtual bool isGoalReachable() = 0;

    UInt8 prev_move;
    UInt8 fut_move;
    std::shared_ptr<Pose> pose;
    std::shared_ptr<Goal> goal;
};

#endif // !BIG_BRAIN_IFC
