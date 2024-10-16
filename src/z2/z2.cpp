#include <ros/ros.h>
#include "autonomy_simulator/RoverPose.h"
#include <iostream>
#include <std_msgs/UInt8.h>

class RoverMovePublisher
{
public:
    RoverMovePublisher(ros::NodeHandle &nh)
    {
        move_pub = nh.advertise<std_msgs::UInt8>("/rover/move", 10);
    }

    void start()
    {
        uint8_t user_input;

        while (user_input != 4)
        {
            int input;
            std::cout << "Please enter a number: ";
            std::cin >> input;
            user_input = static_cast<uint8_t>(input);
            std::cout << "You entered: " << user_input << std::endl;
            if (user_input != 4)
                send(user_input);
        }
    }

    void send(uint8_t &user_input)
    {
        std_msgs::UInt8 msg;
        msg.data = user_input;
        std::cout << msg.data << std::endl;
        move_pub.publish(msg);
    }

private:
    ros::Publisher move_pub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "z2");
    ros::NodeHandle nh;

    ROS_INFO("Hello, this is a z2 node!");
    ROS_WARN("Hello, this is a z2 node!");
    ROS_ERROR("Hello, this is a z2 node!");


    RoverMovePublisher rmp{nh};
    rmp.start();

    return 0;
}
