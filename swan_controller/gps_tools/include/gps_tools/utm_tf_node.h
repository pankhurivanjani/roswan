#ifndef UTM_TF_NODE_H
#define UTM_TF_NODE_H

/*
 * Author: Chen Bainian
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>

#include <gps_common/conversions.h>

class UTM_tf_node{
protected:
    ros::NodeHandle n;

    // Subscriber and publisher for fix, imu and odom
    ros::Subscriber fix_sub, imu_sub;
    ros::Publisher odom_pub;
    
    // TF publisher
    tf::TransformBroadcaster tf_broadcaster;

    // msg to publish
    geometry_msgs::TransformStamped fix_trans;
    nav_msgs::Odometry odom;

    // Private params
    std::string frame_id, child_frame_id;
    bool publish_transform, use_imu_orientation;

    // Recieved fix
    bool acquired_fix;

    // Converted UTM coordinates
    double northing, easting, altitude;
    double rot_cov;
    std::string zone;

    geometry_msgs::Quaternion odom_quat;


public:
    UTM_tf_node();
    UTM_tf_node(int a);
    virtual void init();
    void update_UTM(const sensor_msgs::NavSatFix::ConstPtr& fix);
    void update_orientation(const sensor_msgs::Imu::ConstPtr& imu);
    void pub_tf();

};

#endif
