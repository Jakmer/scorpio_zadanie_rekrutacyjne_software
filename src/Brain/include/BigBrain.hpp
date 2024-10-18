
#ifndef BIG_BRAIN
#define BIG_BRAIN

#include "BigBrainIfc.hpp" 

using UInt8 = std_msgs::UInt8;

class BigBrain : public BigBrainIfc
{
public:
    BigBrain() = default;
    virtual ~BigBrain() = default;

private:
    UInt8 think(Pose &pose, Goal &goal) override;
};

#endif // !BIG_BRAIN
