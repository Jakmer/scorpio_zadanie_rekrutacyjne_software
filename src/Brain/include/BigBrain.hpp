#ifndef BIG_BRAIN
#define BIG_BRAIN

#include "BigBrainIfc.hpp" 
#include "Graph.hpp"
#include <ros/ros.h>

using UInt8 = std_msgs::UInt8;

class BigBrain : public BigBrainIfc
{
public:
    BigBrain(std::shared_ptr<Pose> &posePtr, std::shared_ptr<Goal> &goalPtr, ros::NodeHandle &nh);
    virtual ~BigBrain() = default;

private:
    virtual UInt8 think() override;
    virtual bool validateMove(UInt8 &move) override;
    virtual bool isGoalReachable() override;
    void preprocessMap();

    std::optional<Graph> graph;
};

#endif // !BIG_BRAIN
