#include "PathSolver.hpp"
#include "ros/init.h"
#include <ros/ros.h>
#include <memory>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_solver");

    std::unique_ptr<ps::PathSolver> solver = std::make_unique<ps::PathSolver>();
    solver->run();

    ros::spin();

    return 0;
}
