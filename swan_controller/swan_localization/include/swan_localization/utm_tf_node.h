#ifndef UTM_TF_NODE_H
#define UTM_TF_NODE_H

/*
 * Author: Chen Bainian
 * E-mail: brian97cbn@gmail.com
 * Version: 1.2
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>

#include <gps_common/conversions.h>

class UTM_tf_node{
protected:
    ros::NodeHandle n;

    // Subscriber and publisher for fix, imu and odom
    ros::Subscriber fix_sub, imu_sub;
    ros::Publisher odom_pub;
    
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped fix_trans;

    // msg to publish
    geometry_msgs::Quaternion qt;
    nav_msgs::Odometry odom;

    // Private params
    std::string frame_id, child_frame_id, map_frame_id;
    bool publish_transform, use_imu_orientation, use_map_frame;

    // Recieved fix
    bool acquired_fix;

    // Converted UTM coordinates
    double northing, easting, altitude;
    double map_northing, map_easting;
    double rot_cov;
    std::string zone;

    geometry_msgs::Quaternion odom_quat;


public:
    UTM_tf_node();
    virtual ~UTM_tf_node();
    virtual void init();
    void update_UTM(const sensor_msgs::NavSatFix::ConstPtr& fix);
    void update_orientation(const sensor_msgs::Imu::ConstPtr& imu);
    void pub_tf();
    void reframed_coordinate_to_map();

};

#endif
