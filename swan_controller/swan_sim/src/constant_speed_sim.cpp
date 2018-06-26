#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <boost/assign.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include "swan_sim/base_config.h"

Base_Odom::Base_Odom(double _forward_speed = 0.5)
    : n(), r(20.0)
{
    ros::param::param<std::string>("~frame_id", frame_id, "odom");
    ros::param::param<std::string>("~child_frame_id", child_frame_id, "base_link");
    ros::param::param<bool>("~send_transform", send_transform, false);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    compass_sub = n.subscribe("imu", 1, &Base_Odom::update_yaw, this);
    
    last_time = ros::Time::now();
    current_time = ros::Time::now();

    vx = _forward_speed;
    vth = 0;
    x = 0;
    y = 0;
    z = 0;
    last_th = 0;
    th = 0;
    t = 0;
    dt = 0;

    ros::Duration(10).sleep();
    while(n.ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();
        update_position();
        pub_tf();
        pub_odom();
        last_time = current_time;
        r.sleep();
    }
}

void Base_Odom::update_yaw(const sensor_msgs::Imu::ConstPtr& msg){
    tf::Quaternion heading, yaw_offset;
    tf::quaternionMsgToTF(msg->orientation, heading);
    yaw_offset = tf::createQuaternionFromYaw(M_PI / 2);
    heading *= yaw_offset;
    th = tf::getYaw(heading);

}


void Base_Odom::update_position(){
    if(dt && dt < 1){
        double dx = vx * cos(th) * dt;
        double dy = vx * sin(th) * dt;
        t += dt;
        x += dx;
        y += dy;
        vth = (th - last_th) / dt;
        last_th = th;
    }
}

void Base_Odom::pub_tf(){
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;
    odom_trans.transform.rotation = odom_quat;
    if(send_transform)
        odom_broadcaster.sendTransform(odom_trans);
}

void Base_Odom::pub_odom(){
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    odom.header.stamp = current_time;
    odom.header.frame_id = frame_id;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation = odom_quat;
/*    odom.pose.covariance =       boost::assign::list_of(1e-3) (0) (0) (0) (0) (0)
    (0) (1e-3) (0) (0) (0) (0)
    (0) (0)  (1e6) (0) (0) (0)
    (0) (0) (0)  (1e6) (0) (0) 
    (0) (0) (0) (0)  (1e6) (0)
    (0) (0) (0) (0) (0)  (1e3);
*/
    odom.child_frame_id = child_frame_id;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vth;
/*    odom.twist.covariance =      boost::assign::list_of(1e-3) (0) (0) (0) (0) (0)
    (0) (1e-3) (0) (0) (0) (0)
    (0) (0)  (1e6) (0) (0) (0)
    (0) (0) (0)  (1e6) (0) (0) 
    (0) (0) (0) (0)  (1e6) (0)
    (0) (0) (0) (0) (0)  (1e3);
*/ 
    odom_pub.publish(odom);
}



int main(int argc, char** argv){
    ros::init(argc, argv, "base_odom_node");
    
    double forward_speed;

    ros::param::param<double>("~forward_speed", forward_speed, 0.5);
    Base_Odom base_odom(forward_speed);
    
    return 0;
}
