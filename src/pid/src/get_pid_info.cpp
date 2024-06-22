#include "get_pid_info.hpp"

namespace get_pid_info
{
   PID::PID()
    {
    }

double PID::calc(double error) 
{
    ErrorVector.push_back(error);
    return (kp * ErrorVector[ErrorVector.size()-1]) + (kd * calcDerivative()) + (ki * calcIntegral(error)); 
}

double PID::calcIntegral(double error){
    LastIntegral += error;
    return LastIntegral;
}

double PID::calcDerivative(){
    return ErrorVector[ErrorVector.size()-1] -ErrorVector[ErrorVector.size()-2];
}
}
