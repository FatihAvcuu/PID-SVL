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
        if (odom_datas_x.size() == 0)
        {
            svl_msg.header.stamp = ros::Time::now();
            svl_msg.header.frame_id = "base_link"; 
            svl_msg.twist_cmd.twist.linear.x = 0; 
            svl_msg.twist_cmd.twist.angular.z = 0;
            vehicle_cmd_pub.publish(svl_msg);
            return;
        }

        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        yaw = tf::getYaw(msg->pose.pose.orientation);
        min_i = 0;
        for (size_t i = 0; i < odom_datas_x.size(); ++i)
        {
            dist = sqrt(pow(odom_datas_x[i] - x, 2) + pow(odom_datas_y[i] - y, 2));
            if (dist < minValue)
            {
                minValue = dist;
                min_i = i;
            }
        }
        if(min_i == 0){
            min_i +=1;
        }
        dx = odom_datas_x[min_i] - odom_datas_x[min_i - 1];
        dy = odom_datas_y[min_i] - odom_datas_y[min_i - 1];
        c_error = (dy * (x - odom_datas_x[min_i]) - dx * (y - odom_datas_y[min_i])) / sqrt(dx * dx + dy * dy);
        path_yaw = atan2(dy, dx);
        yaw_error = path_yaw - yaw;
        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
        steering_angle = stanley_ctr.calc(k, car_velocity, c_error) + 1.05*yaw_error;
        svl_msg.header.stamp = ros::Time::now();
        svl_msg.header.frame_id = "base_link"; 
        svl_msg.twist_cmd.twist.linear.x = car_velocity; 
        svl_msg.twist_cmd.twist.angular.z = steering_angle;
        vehicle_cmd_pub.publish(svl_msg);
        std_msgs::String output;
        std::stringstream ss;
        ss << "Received Odometry Message: Seq: " << msg->header.seq;
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

    bool SUBS::readParameters()
    {
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
