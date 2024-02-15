#include "get_pid_info.hpp"


namespace get_pid_info
{
   PID::PID()
    {
        // Yapıcı fonksiyonun tanımı
    }

/*
bool PID::readParameters()
{
    if (!m_node_handle.getParam("kp", kp))
    {
        return false;
    }
    if (!m_node_handle.getParam("ki", ki))
    {
        return false;
    }
    if (!m_node_handle.getParam("kd", kd))
    {
        return false;
    }
    return true;
}
*/

    /*
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }
    */

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
