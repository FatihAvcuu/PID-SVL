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
#include "pure_pursite.hpp"
#include <autoware_msgs/VehicleCmd.h>
#include <fstream>
#include <iterator>
#include <chrono>

namespace subscriber
{
class SUBS
{
  public:
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
  
    double car_velocity,lp,axs,alpha,ld,c_error,desired,steering_angle,roll, pitch, yaw,xa0,ya0,xa2,ya2,xa,ya,seq,x,y,z;
    double last_i = 0;
    int last_min_i = 1;
    int min_i=0;
    int min_i2=0;
    double minValue=INT_MAX;
    double minValue2=INT_MAX;
    std::string odom_topic;
    autoware_msgs::VehicleCmd svl_msg;

  bool readParameters();
    get_pure_pursite::PP pure_pursite_ctr;
    std::chrono::time_point<std::chrono::steady_clock> baslangic_zamani;
    ros::Subscriber sub;
    ros::Subscriber sim_sub;
    ros::Subscriber path_sub;   
    ros::Publisher marker_pub;
    ros::Publisher vehicle_cmd_pub;
    ros::Publisher m_parameter_publisher;
};
} 
