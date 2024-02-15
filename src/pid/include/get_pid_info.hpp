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
    double kp , ki,  kd;

    double calc(double error);


  private:

    double calcIntegral(double error);
    double calcDerivative();

    double LastIntegral = 0;
 
    std::vector<double> ErrorVector = {0};
};
} 
#endif