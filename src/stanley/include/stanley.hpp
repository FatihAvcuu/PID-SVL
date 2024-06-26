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


namespace get_stanley
{
class SC
{
  public:
    SC();
    double calc(double k,double vel,double e_error);


  private:
};
} 
#endif