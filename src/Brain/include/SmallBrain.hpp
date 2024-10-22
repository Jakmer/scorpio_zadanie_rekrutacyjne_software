
#ifndef SMALL_BRAIN
#define SMALL_BRAIN

#include "BigBrainIfc.hpp"

using UInt8 = std_msgs::UInt8;

class SmallBrain : public BigBrainIfc
{
public:
    SmallBrain(std::shared_ptr<Pose> &posePtr, std::shared_ptr<Goal> &goalPtr);
    virtual ~SmallBrain() = default;

private:
    virtual UInt8 think() override;
    virtual bool validateMove(UInt8 &move) override;
    virtual bool isGoalReachable() override;
};

#endif // !SMALL_BRAIN
