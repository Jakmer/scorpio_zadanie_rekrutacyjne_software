#include "include/BigBrain.hpp" 
#include "autonomy_simulator/GetMap.h"

using UInt8 = std_msgs::UInt8;

BigBrain::BigBrain(std::shared_ptr<Pose> &posePtr, std::shared_ptr<Goal> &goalPtr, ros::NodeHandle &nh) : BigBrainIfc(posePtr, goalPtr)
{
    auto client = nh.serviceClient<autonomy_simulator::GetMap>("/get_map");

    autonomy_simulator::GetMap srv;
    std::vector<int8_t> mapList;

    if (client.call(srv)) {
        ROS_INFO("Successfully called service /get_map");

        for(const auto &i : srv.response.data)
        {
            mapList.push_back(i);
        }
    } else {
        ROS_ERROR("Failed to call service /get_map");
    }

    graph = Graph(mapList);
    preprocessMap();
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

void BigBrain::preprocessMap()
{
    /*graph->printGraph(); */
    auto moves = graph->getShortestPath(*pose, *goal);

    ROS_INFO("Printing moves: ");
    for(const auto &move : moves)
    {
        std::cout<<move<<" ";
    }
  
    std::cout<<std::endl;
}
