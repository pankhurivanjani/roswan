#ifndef CAPTAIN_H
#define CAPTAIN_H

/*
 * Author: Chen Bainian
 * E-mail: brian97cbn@gmail.com
 * Version: 1.0
 *
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/LinearMath/Transform.h>
#include <boost/thread.hpp>

class Captain
{
protected:
    ros::NodeHandle n;
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    nav_msgs::Odometry odom;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;

public:
    Captain();
    ~Captain();
    const void run();
    const void simple_point_mission(const std::vector<std::vector<double> >& _mission_points);
    const void loiter();
    void odom_callback(const nav_msgs::OdometryConstPtr& msg);
};
#endif
