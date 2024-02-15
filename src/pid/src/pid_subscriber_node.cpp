#include <ros/ros.h>
#include "pid_subscriber.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pid_subscriber");
    ros::NodeHandle node_handle("~");

    pid_subscriber::PID_SUBS pid(node_handle);

    ros::spin();
    return 0;
}
