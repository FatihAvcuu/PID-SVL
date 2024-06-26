#include "subscriber.hpp"
#include "pure_pursite.hpp"

namespace subscriber
{
SUBS::SUBS(ros::NodeHandle& t_node_handle)
  : m_node_handle(t_node_handle)
{

    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }
    vehicle_cmd_pub = m_node_handle.advertise<autoware_msgs::VehicleCmd>("/vehicle_cmd", 10);
    marker_pub = m_node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    sim_sub = m_node_handle.subscribe(odom_topic, 10, &SUBS::SimOdomCallback, this);
    path_sub = m_node_handle.subscribe("/path_publisher/ShortestPath", 10, &SUBS::PathCallback, this);
    m_parameter_publisher = m_node_handle.advertise<std_msgs::String>("/SUBS", 10);
    ROS_INFO("Successfully launched node Subscriber.");
    baslangic_zamani = std::chrono::steady_clock::now();

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

void SUBS::SimOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
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
    x =   msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z =  msg->pose.pose.position.z;
    min_i=0;
    for (int i = 0; i < odom_datas_x.size(); i++)
    {
        xa = odom_datas_x[i] - x;
        ya = odom_datas_y[i] - y;
        if (minValue > abs(lp-sqrt(xa*xa+ya*ya)))
        {
            minValue= abs(lp-sqrt(xa*xa+ya*ya));
            min_i = i;
            last_min_i = i;
        }
        if (minValue2 > sqrt(xa*xa+ya*ya) )
        {
            minValue2= sqrt(xa*xa+ya*ya);
            min_i2 = i;
        }
    }
    minValue = INT_MAX;
    minValue2 = INT_MAX;
    last_i = min_i;
    xa0 = odom_datas_x[min_i] - x;
    ya0 = odom_datas_y[min_i] - y;
    xa2 = odom_datas_x[min_i] - x;
    ya2 = odom_datas_y[min_i] - y;
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
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    alpha = atan(ya2 / xa2) - yaw;
    if(alpha > M_PI_2) { alpha -= M_PI;}
    if(alpha < -M_PI_2) { alpha += M_PI;}
    ld= sqrt(xa2*xa2+ya2*ya2);
    c_error= sqrt(xa0*xa0+ya0*ya0);
    if (yaw < 0) {
        yaw += 2 * M_PI;
    }
    if (min_i == odom_datas_x.size()-1 && odom_datas_x.size() < 10)
    {
        odom_datas_x.clear();
        odom_datas_y.clear();
    }
    svl_msg.header.stamp = ros::Time::now();
    svl_msg.header.frame_id = "base_link"; 
    svl_msg.twist_cmd.twist.linear.x = car_velocity; 
    steering_angle =pure_pursite_ctr.calc(alpha,ld,axs)*4.2;
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

    x = odom_datas_x[min_i2];
    y = odom_datas_y[min_i2];

    if(min_i2 == 0){
        min_i2 +=1;
    }
    yaw = atan2(odom_datas_y[min_i2]-odom_datas_y[min_i2-1],odom_datas_x[min_i2]-odom_datas_x[min_i2-1]);
    while(true){
    if(yaw > M_PI_2) { yaw -= M_PI;}
    else if(yaw < -M_PI_2) { yaw += M_PI;}
    else{break;}
    }
    xa2 = odom_datas_x[min_i] - x;
    ya2 = odom_datas_y[min_i] - y;
    ld= sqrt(xa2*xa2+ya2*ya2);
    alpha = atan(ya2 / xa2) - yaw;
    if(alpha > M_PI_2) { alpha -= M_PI;}
    if(alpha < -M_PI_2) { alpha += M_PI;}

    desired = pure_pursite_ctr.calc(alpha,ld,axs)*4.2;

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

    std::ofstream dosya("/home/fatih/Desktop/ilayda/PID_SVL/src/pure_pursuit/include/error_pp.csv", std::ios::app);
    if (dosya.is_open()) {
        dosya << saniye << "." << milisaniye  << ";" << steering_angle << ";" <<  desired << std::endl;
        dosya.close();
    } else {
        std::cout << "Dosya acilamadi" << std::endl;
    }

    std::cout << desired << " a " << steering_angle << std::endl;
}

void SUBS::PathCallback(const nav_msgs::Path::ConstPtr& msg)
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

bool SUBS::readParameters(){
    if (!m_node_handle.getParam("car_velocity", car_velocity))
    {
        return false;
    }
    if (!m_node_handle.getParam("lp", lp))
    {
        return false;
    }
    if (!m_node_handle.getParam("axs", axs))
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

