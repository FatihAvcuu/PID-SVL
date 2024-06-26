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
    wheel_pid.kp=kp1;
    wheel_pid.ki=ki1;
    wheel_pid.kd=kd1;
    desired_pid.kp = kp1;
    desired_pid.kd = kd1;
    desired_pid.ki = ki1;
    baslangic_zamani = std::chrono::steady_clock::now();
    vehicle_cmd_pub = m_node_handle.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);
    marker_pub = m_node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    sim_sub = m_node_handle.subscribe(odom_topic, 10, &PID_SUBS::SimOdomCallback, this);
    path_sub = m_node_handle.subscribe("/path_publisher/ShortestPath", 10, &PID_SUBS::PathCallback, this);
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
    marker2.scale.x = 0.3;
    marker2.scale.y = 0.3;
    marker2.scale.z = 0.3;
    marker2.color.r = 0.0f;
    marker2.color.g = 0.0f;
    marker2.color.b = 1.0f;
    marker2.color.a = 1.0;
}

void PID_SUBS::SimOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
     if (odom_datas_x.size()==0)
    {
        svl_msg.header.stamp = ros::Time::now();
        svl_msg.header.frame_id = "base_link"; 
        svl_msg.twist_cmd.twist.linear.x = 0; 
        svl_msg.twist_cmd.twist.angular.z = 0;
        vehicle_cmd_pub.publish(svl_msg);
        return;
    }
    seq = msg->header.seq;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
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

    min_i=0;
    for (int i = 0; i < odom_datas_x.size(); i++)
    {
        xa = odom_datas_x[i] - x;
        ya = odom_datas_y[i] - y;
        if (minValue > abs(sqrt(xa*xa+ya*ya)) )
        {
            minValue= abs(sqrt(xa*xa+ya*ya));
            min_i = i;
            last_min_i = i;
        }
        
    }
    minValue = INT_MAX;
    last_i = min_i;
    xa2 = odom_datas_x[min_i+lp] - x;
    ya2 = odom_datas_y[min_i+lp] - y;
    
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    alpha = atan(ya2 / xa2) - yaw;
    if(alpha > M_PI_2) { alpha -= M_PI;}
    if(alpha < -M_PI_2) { alpha += M_PI;}
    if (yaw < 0) {
        yaw += 2 * M_PI;
    }
    if (min_i == odom_datas_x.size()-1 && odom_datas_x.size() < 10)
    {
        odom_datas_x.clear();
        odom_datas_y.clear();
    }

    marker2.id = 1010;
    marker2.pose.position.x = odom_datas_x[min_i];
    marker2.pose.position.y = odom_datas_y[min_i];
    marker2.pose.position.z = 0;
    marker2.pose.orientation.x = msg->pose.pose.orientation.x;
    marker2.pose.orientation.y = msg->pose.pose.orientation.y;
    marker2.pose.orientation.z = msg->pose.pose.orientation.z;
    marker2.pose.orientation.w = msg->pose.pose.orientation.w;
    marker2.lifetime = ros::Duration();
    marker_pub.publish(marker2);

    svl_msg.header.stamp = ros::Time::now();
    svl_msg.header.frame_id = "base_link"; 
    svl_msg.twist_cmd.twist.linear.x = car_velocity; 
    steering_angle=wheel_pid.calc(alpha);
    svl_msg.twist_cmd.twist.angular.z = steering_angle;
    vehicle_cmd_pub.publish(svl_msg);
    std_msgs::String output;
    std::stringstream ss;
    ss << "Received Odometry Message: Seq: ";;
    output.data = ss.str();
    m_parameter_publisher.publish(output);

    //desired angle
    auto simdi = std::chrono::steady_clock::now();
    auto gecen_sure_saniye = std::chrono::duration_cast<std::chrono::seconds>(simdi - baslangic_zamani);
    auto gecen_sure_milisaniye = std::chrono::duration_cast<std::chrono::milliseconds>(simdi - baslangic_zamani);
    auto saniye = gecen_sure_saniye.count();
    auto milisaniye = gecen_sure_milisaniye.count();
    if(min_i == 0){
        min_i +=1;
    }
    x = odom_datas_x[min_i];
    y = odom_datas_y[min_i];
    yaw = atan2(odom_datas_y[min_i]-odom_datas_y[min_i-1],odom_datas_x[min_i]-odom_datas_x[min_i-1]);
    while(true){
    if(yaw > M_PI_2) { yaw -= M_PI;}
    else if(yaw < -M_PI_2) { yaw += M_PI;}
    else{break;}
    }
    xa2 = odom_datas_x[min_i+lp] - x;
    ya2 = odom_datas_y[min_i+lp] - y;

    alpha = atan(ya2 / xa2) - yaw;
    if(alpha > M_PI_2) { alpha -= M_PI;}
    if(alpha < -M_PI_2) { alpha += M_PI;}
    if (yaw < 0) {
        yaw += 2 * M_PI;
    }
    desired = desired_pid.calc(alpha);

    if (desired>2)
    {
        desired=2;
    }
    if (desired<-2)
    {
        desired=-2;
    }
    
    if (steering_angle > 2)
    {
        steering_angle=2;
    }
    if (steering_angle < -2)
    {
        steering_angle=-2;
    }
    if (isnan(desired))
    {
        desired=0;
    }
    std::ofstream dosya("/home/fatih/Desktop/ilayda/PID_SVL/src/pid/include/error_pid.csv", std::ios::app);
    if (dosya.is_open()) {
        dosya << saniye << "." << milisaniye  << ";" << steering_angle << ";" <<  desired << std::endl;
        dosya.close();
    } else {
        std::cout << "Dosya acilamadi" << std::endl;
    }

    std::cout << desired << " a " << steering_angle << std::endl;
}

void PID_SUBS::PathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    odom_datas_y.clear();
    odom_datas_x.clear();
    if (msg->poses.size() > 40)
    {
    for (size_t i = 0; i < 40; ++i)
    {
        odom_datas_x.push_back(msg->poses[i].pose.position.x);
        odom_datas_y.push_back(msg->poses[i].pose.position.y);
    }
    }
    else
    {       
    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
        odom_datas_x.push_back(msg->poses[i].pose.position.x);
        odom_datas_y.push_back(msg->poses[i].pose.position.y);
    }
    }
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
    if (!m_node_handle.getParam("lp", lp))
    {
        return false;
    }
    if (!m_node_handle.getParam("odom_topic", odom_topic))
    {
        return false;
    }
    return true;
}

}
