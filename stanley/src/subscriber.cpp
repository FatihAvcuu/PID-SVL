#include "subscriber.hpp"
#include "stanley.hpp"



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
        autoware_msgs::VehicleCmd svl_msg;
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

    /*
    int min_i=0;
    for (int i = 0; i < odom_datas_x.size(); i++)
    {
        double xa = odom_datas_x[i] - x;
        double ya = odom_datas_y[i] - y;
        if (minValue > sqrt(xa*xa+ya*ya) )
        {
            minValue= sqrt(xa*xa+ya*ya);
            min_i = i;
        }
        
    }
    minValue = INT_MAX;
    last_i = min_i;
    */
    int lp=5;
    int min_i=0;
    int min_i2=0;
    for (int i = 0; i < odom_datas_x.size(); i++)
    {
        double xa = odom_datas_x[i] - x;
        double ya = odom_datas_y[i] - y;
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
    double xa0 = odom_datas_x[min_i] - x;
    double ya0 = odom_datas_y[min_i] - y;
    double xa2 = odom_datas_x[min_i] - x;
    double ya2 = odom_datas_y[min_i] - y;
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
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    if (yaw < 0) {
        yaw += 2 * M_PI;
    }
    
    if (min_i == odom_datas_x.size()-1 && odom_datas_x.size() < 10)
    {
        odom_datas_x.clear();
        odom_datas_y.clear();
    }

    double yaw_path = std::atan2(xa0, ya0);


        double test2= atan(((y-1*sin(yaw))-odom_datas_y[min_i2]) / ((x-1*cos(yaw))-odom_datas_x[min_i2])) - yaw;
    while(true){
    if(test2 > M_PI_2) { test2 -= M_PI;}
    else if(test2 < -M_PI_2) { test2 += M_PI;}
    else{break;}
    }


    if( odom_datas_x[min_i2+1] == odom_datas_x[min_i2] || odom_datas_y[min_i2+1] == odom_datas_y[min_i2]){
        c_error= last_c_error;
    }
    else if (  0  > test2){
        last_c_error= -(abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2]))*((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])))+1));
        c_error= -(abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2]))*((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])))+1));
    }
    else
    {
        last_c_error= (abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2]))*((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])))+1));
        c_error= (abs(((odom_datas_y[min_i2] - odom_datas_y[min_i2+1])/(odom_datas_x[min_i2] - odom_datas_x[min_i2+1]))*((x) - odom_datas_x[min_i2])+odom_datas_y[min_i2+1] - (y))/sqrt((((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2]))*((odom_datas_y[min_i2+1] - odom_datas_y[min_i2])/(odom_datas_x[min_i2+1] - odom_datas_x[min_i2])))+1));
    }
    double m1=((odom_datas_y[min_i-1] - odom_datas_y[min_i])/(odom_datas_x[min_i-1] - odom_datas_x[min_i]));
    double test3=atan2(odom_datas_y[min_i-1] - odom_datas_y[min_i],odom_datas_x[min_i-1] - odom_datas_x[min_i]);
    
    while(true){
    if(test3 > M_PI_2) { test3 -= M_PI;}
    else if(test3 < -M_PI_2) { test3 += M_PI;}
    else{break;}
    }/*
       if (test3 < 0) {
        test3 += 2 * M_PI;
    }*/
    

    if(!isnan(test3)){old_test3=test3;}

    while(true){
    if(yaw > M_PI_2) { yaw -= M_PI;}
    else if(yaw < -M_PI_2) { yaw += M_PI;}
    else{break;}
    }

    if(isnan(test3)){test3=old_test3;}

        double m2=atan((yaw-test3)/(1+(yaw*test3)));
    if(atan((yaw-test3)/(1+(yaw*test3)))>1){
        m2=atan((yaw-test3)/(1+(yaw*test3)))-1;
    }
    else{
        m2=atan((yaw-test3)/(1+(yaw*test3)));
    }

    while(true){
    if(m2 > M_PI_2) { m2 -= M_PI;}
    else if(m2 < -M_PI_2) { m2 += M_PI;}
    else{break;}
    }


    /****************************/

    waytrack= atan(((y+10*sin(yaw)*20)-odom_datas_y[min_i]) / ((x+10*cos(yaw)*20)-odom_datas_x[min_i])) - yaw;
    while(true){
    if(waytrack > M_PI_2) { waytrack -= M_PI;}
    else if(waytrack < -M_PI_2) { waytrack += M_PI;}
    else{break;}
    }

    if (  0  < waytrack)
    {
        c_calc= -(abs(((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1]))*((x+lp*cos(yaw)*10) - odom_datas_x[min_i])    +odom_datas_y[min_i] - (y+lp*sin(yaw)*10))/sqrt((((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1]))*((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1])))+1));
    }
    else
    {
        c_calc= abs(((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1]))*((x+lp*cos(yaw)*10) - odom_datas_x[min_i])    +odom_datas_y[min_i] - (y+lp*sin(yaw)*10))/sqrt((((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1]))*((odom_datas_y[min_i] - odom_datas_y[min_i-1])/(odom_datas_x[min_i] - odom_datas_x[min_i-1])))+1);
    }
    double aaabbb = (test3-yaw)/(1+test3*yaw);
    while(true){
    if(aaabbb > M_PI_2) { aaabbb -= M_PI;}
    else if(aaabbb < -M_PI_2) { aaabbb += M_PI;}
    else{break;}
    }
    std::cout << test3*(180/M_PI) << " a " <<  yaw*(180/M_PI)  << " b " << aaabbb  << " c "<< std::endl;

    autoware_msgs::VehicleCmd svl_msg;
    svl_msg.header.stamp = ros::Time::now();
    svl_msg.header.frame_id = "base_link"; 
    svl_msg.twist_cmd.twist.linear.x = car_velocity; 
    //svl_msg.twist_cmd.twist.angular.z = stanley_ctr.calc(k,car_velocity,c_error)-2*aaabbb;
    vehicle_cmd_pub.publish(svl_msg);
    std_msgs::String output;
    std::stringstream ss;
    ss << "Received Odometry Message: Seq: ";;
    output.data = ss.str();
    m_parameter_publisher.publish(output);
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
    if (!m_node_handle.getParam("k", k))
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

