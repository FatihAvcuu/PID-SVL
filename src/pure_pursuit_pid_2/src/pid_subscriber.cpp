#include "pid_subscriber.hpp"
#include "get_pid_info.hpp"
#include "pure_pursite.hpp"

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
    baslangic_zamani = std::chrono::steady_clock::now();

    desired_pid.kp=kp1;
    desired_pid.ki=ki1;
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

    marker3.header.frame_id = "map";
    marker3.header.stamp = ros::Time::now();
    marker3.ns = "basic_shapes";
    marker3.type = visualization_msgs::Marker::SPHERE;
    marker3.action = visualization_msgs::Marker::ADD;
    marker3.scale.x = 0.3;
    marker3.scale.y = 0.3;
    marker3.scale.z = 0.3;
    marker3.color.r = 0.0f;
    marker3.color.g = 1.0f;
    marker3.color.b = 0.0f;
    marker3.color.a = 1.0;
}

void PID_SUBS::SimOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
     if (odom_datas_x.size()<2)
    {
        svl_msg.header.stamp = ros::Time::now();
        svl_msg.header.frame_id = "base_link"; 
        svl_msg.twist_cmd.twist.linear.x = 0; 
        svl_msg.twist_cmd.twist.angular.z = 0;
        vehicle_cmd_pub.publish(svl_msg);
        return;
    }
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

    min_i=0;
    min_i2=0;
    for (int i = 0; i < odom_datas_x.size(); i++)
    {
        xa = odom_datas_x[i] - x;
        ya = odom_datas_y[i] - y;
        if (minValue > abs(lp-sqrt(xa*xa+ya*ya)) )
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
    xa0 = odom_datas_x[min_i2] - x;
    ya0 = odom_datas_y[min_i2] - y;
    xa2 = odom_datas_x[min_i] - x;
    ya2 = odom_datas_y[min_i] - y;
    
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    alpha = atan(ya2 / xa2) - yaw;
    if(alpha > M_PI_2) { alpha -= M_PI;}
    if(alpha < -M_PI_2) { alpha += M_PI;}
    ld= sqrt(xa2*xa2+ya2*ya2);
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

    marker3.id = 1011;
    marker3.pose.position.x = x+ld*cos(yaw)*20;
    marker3.pose.position.y = y+ld*sin(yaw)*20;
    marker3.pose.position.z = 0;
    marker3.pose.orientation.x = msg->pose.pose.orientation.x;
    marker3.pose.orientation.y = msg->pose.pose.orientation.y;
    marker3.pose.orientation.z = msg->pose.pose.orientation.z;
    marker3.pose.orientation.w = msg->pose.pose.orientation.w;
    marker3.lifetime = ros::Duration();
    marker_pub.publish(marker3);

    waytrack= atan(((y+ld*sin(yaw)*20)-odom_datas_y[min_i]) / ((x+ld*cos(yaw)*20)-odom_datas_x[min_i])) - yaw;
    while(true){
    if(waytrack > M_PI_2) { waytrack -= M_PI;}
    else if(waytrack < -M_PI_2) { waytrack += M_PI;}
    else{break;}
    }


    waytrack2= atan(((y-1*sin(yaw))-odom_datas_y[min_i2]) / ((x-1*cos(yaw))-odom_datas_x[min_i2])) - yaw;
    while(true){
    if(waytrack2 > M_PI_2) { waytrack2 -= M_PI;}
    else if(waytrack2 < -M_PI_2) { waytrack2 += M_PI;}
    else{break;}
    }
    if (  0  < waytrack)
    {
        c_calc= -(abs(((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1]))*((x+lp*cos(yaw)*10) - odom_datas_x[min_i])    +odom_datas_y[min_i] - (y+lp*sin(yaw)*10))/sqrt((pow(((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1])),2))+1));
    }
    else
    {
        c_calc= abs(((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1]))*((x+lp*cos(yaw)*10) - odom_datas_x[min_i])    +odom_datas_y[min_i] - (y+lp*sin(yaw)*10))/sqrt((pow(((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1])),2))+1);
    }
    if( odom_datas_x[min_i2+1] == odom_datas_x[min_i2] || odom_datas_y[min_i2+1] == odom_datas_y[min_i2]){
        c_error= last_c_error;
    }
    else if (  0  > waytrack2){
        last_c_error= -(abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((pow(((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])),2))+1));
        c_error= -(abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((pow(((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])),2))+1));
    }
    else
    {
        last_c_error= (abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((pow(((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])),2))+1));
        c_error= (abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((pow(((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])),2))+1));
    }
    double anglee = 0;
    svl_msg.header.stamp = ros::Time::now();
    svl_msg.header.frame_id = "base_link"; 
    svl_msg.twist_cmd.twist.linear.x = car_velocity; 
    
    anglee=pure_pursite_ctr.calc(alpha,ld,axs)*4 + wheel_pid.calc(c_error,ld,c_calc);
    svl_msg.twist_cmd.twist.angular.z = anglee;

    vehicle_cmd_pub.publish(svl_msg);
    std_msgs::String output;
    std::stringstream ss;
    ss << "Received Odometry Message: Seq: ";;
    output.data = ss.str();
    m_parameter_publisher.publish(output);

    //desired angle



    x = odom_datas_x[min_i2];
    y = odom_datas_y[min_i2];

    xa0 = odom_datas_x[min_i2] - x;
    ya0 = odom_datas_y[min_i2] - y;
    xa2 = odom_datas_x[min_i] - x;
    ya2 = odom_datas_y[min_i] - y;
    if(min_i2 == 0){
        min_i2 +=1;
    }
    yaw = atan2(odom_datas_y[min_i2]-odom_datas_y[min_i2-1],odom_datas_x[min_i2]-odom_datas_x[min_i2-1]);
    while(true){
    if(yaw > M_PI_2) { yaw -= M_PI;}
    else if(yaw < -M_PI_2) { yaw += M_PI;}
    else{break;}
    }

    alpha = atan(ya2 / xa2) - yaw;
    if(alpha > M_PI_2) { alpha -= M_PI;}
    if(alpha < -M_PI_2) { alpha += M_PI;}
    ld= sqrt(xa2*xa2+ya2*ya2);

    waytrack= atan(((y+ld*sin(yaw)*20)-odom_datas_y[min_i]) / ((x+ld*cos(yaw)*20)-odom_datas_x[min_i])) - yaw;
    while(true){
    if(waytrack > M_PI_2) { waytrack -= M_PI;}
    else if(waytrack < -M_PI_2) { waytrack += M_PI;}
    else{break;}
    }


    waytrack2= atan(((y-1*sin(yaw))-odom_datas_y[min_i2]) / ((x-1*cos(yaw))-odom_datas_x[min_i2])) - yaw;
    while(true){
    if(waytrack2 > M_PI_2) { waytrack2 -= M_PI;}
    else if(waytrack2 < -M_PI_2) { waytrack2 += M_PI;}
    else{break;}
    }
    if (  0  < waytrack)
    {
        c_calc= -(abs(((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1]))*((x+lp*cos(yaw)*10) - odom_datas_x[min_i])    +odom_datas_y[min_i] - (y+lp*sin(yaw)*10))/sqrt((pow(((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1])),2))+1));
    }
    else
    {
        c_calc= abs(((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1]))*((x+lp*cos(yaw)*10) - odom_datas_x[min_i])    +odom_datas_y[min_i] - (y+lp*sin(yaw)*10))/sqrt(((pow(((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1])),2))+1));
    }
    if( odom_datas_x[min_i2+1] == odom_datas_x[min_i2] || odom_datas_y[min_i2+1] == odom_datas_y[min_i2]){
        c_error= last_c_error;
    }
    else if (  0  > waytrack2){
       last_c_error= -(abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((pow(((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])),2))+1));
        c_error= -(abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((pow(((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])),2))+1));
    }
    else
    {
        last_c_error= (abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((pow(((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])),2))+1));
        c_error= (abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt(pow(((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])),2)+1));
    }

     auto simdi = std::chrono::steady_clock::now();
        auto gecen_sure_saniye = std::chrono::duration_cast<std::chrono::seconds>(simdi - baslangic_zamani);
        auto gecen_sure_milisaniye = std::chrono::duration_cast<std::chrono::milliseconds>(simdi - baslangic_zamani);
        auto saniye = gecen_sure_saniye.count();
        auto milisaniye = gecen_sure_milisaniye.count();

        double desired = 0;
        desired = pure_pursite_ctr.calc(alpha,ld,axs)*4 + desired_pid.calc(c_error,ld,c_calc) ;
        if (desired>2)
        {
            desired=2;
        }
        if (desired<-2)
        {
            desired=-2;
        }
        
        if (anglee > 2)
        {
            anglee=2;
        }
        if (anglee < -2)
        {
            anglee=-2;
        }
        if (isnan(desired))
        {
            desired=0;
        }
        
        
        
         std::ofstream dosya("/home/fatih/Desktop/ilayda/PID_SVL/src/pure_pursuit_pid_2/include/error_stanley.csv", std::ios::app);
    if (dosya.is_open()) {
        dosya << saniye << "." << milisaniye  << ";" << anglee << ";" <<  desired << std::endl;
        dosya.close();
    } else {
        std::cout << "Dosya acilamadi" << std::endl;
    }

    std::cout << desired << " a " << anglee << std::endl;

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