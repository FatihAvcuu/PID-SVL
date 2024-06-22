#pragma once
#ifndef PURE_PURSITE_HPP
#define PURE_PURSITE_HPP

#include <ros/ros.h>
#include <string> 
#include <std_msgs/Float64.h> 
#include "std_msgs/String.h"
#include <sstream>
#include <vector>
#include <random>
#include <cmath>
#include <memory>

namespace get_pure_pursite
{
class PP
{
  public:
    PP();
    double calc(double alpha,double ld,double axs);
  private:
    std::vector<double> ErrorVector = {0};
};
} 
#endif