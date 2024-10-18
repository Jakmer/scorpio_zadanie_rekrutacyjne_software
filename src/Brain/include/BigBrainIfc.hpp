#ifndef BIG_BRAIN_IFC
#define BIG_BRAIN_IFC

#include "autonomy_simulator/RoverPose.h"
#include "autonomy_simulator/SetGoal.h"
#include <std_msgs/UInt8.h>

using UInt8 = std_msgs::UInt8;
using Pose = autonomy_simulator::RoverPose;
using Goal = autonomy_simulator::SetGoal;

class BigBrainIfc
{
public:
    BigBrainIfc() = default;
    virtual ~BigBrainIfc() = default;
    UInt8 getBestMove(Pose &pose, Goal &goal){ return think(pose, goal); }

private:
    virtual UInt8 think(Pose &pose, Goal &goal) = 0;
    virtual bool validateMove(UInt8 &move) = 0;

    UInt8 prev_move;
    UInt8 fut_move;
};

#endif // !BIG_BRAIN_IFC
