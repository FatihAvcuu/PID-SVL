#pragma once
#ifndef GET_PID_INFO_HPP
#define GET_PID_INFO_HPP

#include <ros/ros.h>
#include <string> 
#include <std_msgs/Float64.h> 
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <random>
#include <cmath>
#include <memory>

namespace get_pid_info
{
class PID
{
  public:
    PID();
    double kp , ki,sum;
    double calc(double error,double ld,double cerror);
  private:
    double calcIntegral(double error);
    double calcProportional(double cerror);
    double LastIntegral = 0;
    std::vector<double> ErrorVector;
    int maxElements = 8; 
};
} 
#endif