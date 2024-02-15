#include "pid_subscriber.hpp"
#include "get_pid_info.hpp"




namespace pid_subscriber
{
PID_SUBS::PID_SUBS(ros::NodeHandle& t_node_handle)
  : m_node_handle(t_node_handle)
{

    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }


    std::ifstream filex("/home/fatih/Desktop/ilayda/PID_SVL/src/pid/include/x_data.csv");
    std::ifstream filey("/home/fatih/Desktop/ilayda/PID_SVL/src/pid/include/y_data.csv");

 if (!filex) {
        std::cerr << "Dosya açma hatası!" << std::endl;
    }
     if (!filey) {
        std::cerr << "Dosya açma hatası!" << std::endl;
    }
      //  baslangic_zamani = std::chrono::steady_clock::now();
    std::string linex;
    int linexNumber = 1;

    while (std::getline(filex, linex))
    {
        if (linexNumber % 2 != 0) // Satır numarası tek   
        {
            std::istringstream iss(linex);
            double value;

            if (iss >> value)
            {
                odom_datas_x.push_back(value);
            }
        }

        ++linexNumber;
    }


    std::string liney;
    int lineyNumber = 1;
    while (std::getline(filey, liney))
    {
        if (lineyNumber % 2 != 0) // Satır numarası tek ise
        {
            std::istringstream iss(liney);
            double value;

            if (iss >> value)
            {
                odom_datas_y.push_back(value);
            }
        }

        ++lineyNumber;
    }


   // std::cout << odom_datas[1] << std::endl;




    wheel_pid.kp=kp1;
    wheel_pid.ki=ki1;
    wheel_pid.kd=kd1;

    vehicle_cmd_pub = m_node_handle.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);
    marker_pub = m_node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    sim_sub = m_node_handle.subscribe("/odom_sim", 10, &PID_SUBS::SimOdomCallback, this);
    //sub = m_node_handle.subscribe("/odom", 10, &PID_SUBS::odomCallback, this);
    timer_subscriber = m_node_handle.createTimer(ros::Duration(0.1), &PID_SUBS::topicCallback, this);
    m_parameter_publisher = m_node_handle.advertise<std_msgs::String>("/PID_SUBS", 10);
    ROS_INFO("Successfully launched node Subscriber.");

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker2.header.frame_id = "map";
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "basic_shapes";
    marker2.type = visualization_msgs::Marker::SPHERE;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.scale.x = 1.0;
    marker2.scale.y = 1.0;
    marker2.scale.z = 1.0;
    marker2.color.r = 0.0f;
    marker2.color.g = 0.0f;
    marker2.color.b = 1.0f;
    marker2.color.a = 1.0;
}
/*
void PID_SUBS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double seq = msg->header.seq;
    double x =   msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z =  msg->pose.pose.position.z;
    std_msgs::String output;
    std::stringstream ss;
    ss << "Received Odometry Message: Seq: " <<  z << " Pose " << x << "," << y << "," << z;
    output.data = ss.str();
    //m_parameter_publisher.publish(output);
  
    marker.id = seq;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = msg->pose.pose.orientation.x;
    marker.pose.orientation.y = msg->pose.pose.orientation.y;
    marker.pose.orientation.z = msg->pose.pose.orientation.z;
    marker.pose.orientation.w = msg->pose.pose.orientation.w;
    marker.lifetime = ros::Duration();
    markerVector.push_back(marker);
    marker_pub.publish(marker);
    
}
*/
void PID_SUBS::SimOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double seq = msg->header.seq;
    double x =   msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z =  msg->pose.pose.position.z;

  
    marker2.id = 1010;
    marker2.pose.position.x = x;
    marker2.pose.position.y = y;
    marker2.pose.position.z = z;
    marker2.pose.orientation.x = msg->pose.pose.orientation.x;
    marker2.pose.orientation.y = msg->pose.pose.orientation.y;
    marker2.pose.orientation.z = msg->pose.pose.orientation.z;
    marker2.pose.orientation.w = msg->pose.pose.orientation.w;
    marker2.lifetime = ros::Duration();
    marker_pub.publish(marker2);

    double minValue=10000000000;
    int min_i=0;
    for (int i = 0; i < odom_datas_x.size(); i++)
    {
        double xa = odom_datas_x[i] - x;
        double ya = odom_datas_y[i] - y;
        if (minValue > (xa*xa+ya*ya)  &&  abs(last_min_i-i) < 200)
        {
            minValue= xa*xa+ya*ya;
            min_i = i;
            last_min_i = i;
        }
        
    }

    if (min_i == markerVector.size())
    {
        min_i = min_i-21;
    }
    
    double xa2 = odom_datas_x[min_i+20] - x;
    double ya2 = odom_datas_y[min_i+20] - y;

    double delta2 = atan2(-2*ya2 , xa2*xa2+ya2*ya2);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    double alpha = atan(ya2 / xa2) - yaw;



    if(alpha > M_PI_2) { alpha -= M_PI;}
    if(alpha < -M_PI_2) { alpha += M_PI;}


            

    double delta = atan((2*sin(alpha)) / sqrt(xa2*xa2+ya2*ya2));

    
    if (yaw < 0) {
        yaw += 2 * M_PI;
    }
/*

    auto simdi = std::chrono::steady_clock::now();
        auto gecen_sure_saniye = std::chrono::duration_cast<std::chrono::seconds>(simdi - baslangic_zamani);
        auto gecen_sure_milisaniye = std::chrono::duration_cast<std::chrono::milliseconds>(simdi - baslangic_zamani);
        auto saniye = gecen_sure_saniye.count();
        auto milisaniye = gecen_sure_milisaniye.count();


         std::ofstream dosya("/home/fatih/Desktop/ilayda/PID_SVL/src/pid/include/error.csv", std::ios::app);
    if (dosya.is_open()) {
        dosya << saniye << "." << milisaniye  << ";" << delta << std::endl;
        dosya.close();
    } else {
        std::cout << "Dosya acilamadi" << std::endl;
    }

*/

    autoware_msgs::VehicleCmd svl_msg;
    svl_msg.header.stamp = ros::Time::now();
    svl_msg.header.frame_id = "base_link"; 
    svl_msg.twist_cmd.twist.linear.x = car_velocity; 
    svl_msg.twist_cmd.twist.angular.z =   wheel_pid.calc(delta*5); 

    vehicle_cmd_pub.publish(svl_msg);

    std_msgs::String output;
    std::stringstream ss;
    ss << "Received Odometry Message: Seq: " << seq << " Pose " << min_i << " yaw: " << yaw* 180 / M_PI << " delta: " << delta* 1800 / M_PI << " Fark: "<<delta* 180 / M_PI - yaw* 180 / M_PI <<" teker: " << delta2;;
    output.data = ss.str();
    m_parameter_publisher.publish(output);
}


void PID_SUBS::topicCallback(const ros::TimerEvent&)
{

}

bool PID_SUBS::readParameters(){
    if (!m_node_handle.getParam("kp", kp1))
    {
        return false;
    }
    if (!m_node_handle.getParam("ki", ki1))
    {
        return false;
    }
    if (!m_node_handle.getParam("kd", kd1))
    {
        return false;
    }
    if (!m_node_handle.getParam("car_velocity", car_velocity))
    {
        return false;
    }
    return true;
}

}
