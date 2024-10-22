#ifndef BIG_BRAIN
#define BIG_BRAIN

#include "BigBrainIfc.hpp" 

using UInt8 = std_msgs::UInt8;

class BigBrain : public BigBrainIfc
{
public:
    BigBrain(std::shared_ptr<Pose> &posePtr, std::shared_ptr<Goal> &goalPtr);
    virtual ~BigBrain() = default;

private:
    virtual UInt8 think() override;
    virtual bool validateMove(UInt8 &move) override;
    virtual bool isGoalReachable() override;
};

#endif // !BIG_BRAIN
