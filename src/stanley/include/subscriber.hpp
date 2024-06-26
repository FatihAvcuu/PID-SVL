#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <string> 
#include <std_msgs/Float64.h> 
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <random>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "stanley.hpp"
#include <autoware_msgs/VehicleCmd.h>
#include <fstream>
#include <iterator>
#include <chrono>




namespace subscriber
{
class SUBS
{
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    SUBS(ros::NodeHandle& t_node_handle);

  private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void SimOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void PathCallback(const nav_msgs::Path::ConstPtr& msg);


    ros::NodeHandle& m_node_handle;
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker2;
    std::vector<visualization_msgs::Marker> markerVector;
    std::vector<double> odom_datas_x;
    std::vector<double> odom_datas_y;
    tf::TransformBroadcaster br;
    std::string u="w";
  
    double car_velocity,k,c_error,waytrack,c_calc,x,y,yaw,dist,dx,dy,path_yaw,yaw_error,steering_angle,last_c_error;
    double last_i = 0;
    double min_i = 0;
    int last_min_i = 1;
    double minValue=INT_MAX;
    std::string odom_topic;
    bool readParameters();
    get_stanley::SC stanley_ctr;
    std::chrono::time_point<std::chrono::steady_clock> baslangic_zamani;
    autoware_msgs::VehicleCmd svl_msg;
    ros::Subscriber sub;
    ros::Subscriber sim_sub;
    ros::Subscriber path_sub;
    ros::Publisher marker_pub;
    ros::Publisher vehicle_cmd_pub;
    ros::Publisher m_parameter_publisher;
};
} 
