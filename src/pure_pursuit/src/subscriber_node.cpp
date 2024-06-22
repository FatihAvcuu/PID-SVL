#include <ros/ros.h>
#include "subscriber.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle node_handle("~");
    subscriber::SUBS pure_pursuit(node_handle);
    ros::spin();
    return 0;
}
