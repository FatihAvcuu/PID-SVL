#include "get_pid_info.hpp"

namespace get_pid_info
{
   PID::PID()
    {
    }

double PID::calc(double error,double ld,double cerror) 
{
    return (kp * calcProportional(cerror)) + (ki * calcIntegral(error)); 
}

double PID::calcIntegral(double error){
    if (ErrorVector.size() >= maxElements) {
        ErrorVector.erase(ErrorVector.begin());
    }
        ErrorVector.push_back(error);
    sum = 0;
    for (int i = 0; i < ErrorVector.size(); i++) {
        sum += ErrorVector[i];
    }
    return sum;
}

double PID::calcProportional(double cerror){
    return cerror;
}

}
