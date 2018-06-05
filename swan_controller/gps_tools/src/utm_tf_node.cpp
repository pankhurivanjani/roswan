#include <gps_tools/utm_tf_node.h>
using namespace gps_common;


UTM_tf_node::UTM_tf_node(){
    ;
}

UTM_tf_node::UTM_tf_node(int a)
:n(), acquired_fix(false)
{
    init();
}

void UTM_tf_node::init(){
    int frequency;
    ros::param::param<std::string>("~frame_id", frame_id, "odom");
    ros::param::param<std::string>("~child_frame_id", child_frame_id, "base_link");
    ros::param::param<bool>("~publish_transform", publish_transform, true);
    ros::param::param<bool>("~use_imu_orientation", use_imu_orientation, false);
    ros::param::param<int>("~frequency", frequency, 20);
    ros::param::param<double>("~rot_covariance", rot_cov, 99999.0);
    ros::Rate r(frequency);


    fix_sub = n.subscribe("fix", 1, &UTM_tf_node::update_UTM, this);
    if(use_imu_orientation)
        imu_sub = n.subscribe("imu", 1, &UTM_tf_node::update_orientation, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    
    fix_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    while(n.ok()){
        ros::spinOnce();
        if(publish_transform){
            pub_tf();
        }
        r.sleep();
    }

}

void UTM_tf_node::update_UTM(const sensor_msgs::NavSatFix::ConstPtr& fix){
    if(fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
        ROS_INFO("No fix.");
        return;
    }
    
    if(fix->header.stamp == ros::Time(0)) {
        return;
    }
    LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
    altitude = fix->altitude;

    if (odom_pub) {
        odom.header.stamp = ros::Time::now();
        if (frame_id.empty())
            odom.header.frame_id = fix->header.frame_id;
        else
            odom.header.frame_id = frame_id;

        odom.child_frame_id = child_frame_id;
    
        odom.pose.pose.position.x = easting;
        odom.pose.pose.position.y = northing;
        odom.pose.pose.position.z = altitude;
        
        odom.pose.pose.orientation = fix_trans.transform.rotation;
        
        // Use ENU covariance to build XYZRPY covariance
        boost::array<double, 36> covariance = {{
            fix->position_covariance[0],
            fix->position_covariance[1],
            fix->position_covariance[2],
            0, 0, 0,
            fix->position_covariance[3],
            fix->position_covariance[4],
            fix->position_covariance[5],
            0, 0, 0,
            fix->position_covariance[6],
            fix->position_covariance[7],
            fix->position_covariance[8],
            0, 0, 0,
            0, 0, 0, rot_cov, 0, 0,
            0, 0, 0, 0, rot_cov, 0,
            0, 0, 0, 0, 0, rot_cov
        }};
    
        odom.pose.covariance = covariance;
    
        odom_pub.publish(odom);
    }
    if(publish_transform){
        pub_tf();
    }
}

void UTM_tf_node::pub_tf(){
    fix_trans.header.stamp = ros::Time::now();
    fix_trans.header.frame_id = frame_id;
    fix_trans.child_frame_id = child_frame_id;
    fix_trans.transform.translation.x = easting;
    fix_trans.transform.translation.y = northing;
    fix_trans.transform.translation.z = altitude;
    tf_broadcaster.sendTransform(fix_trans);
}

void UTM_tf_node::update_orientation(const sensor_msgs::Imu::ConstPtr& imu){
    tf::Quaternion heading, yaw_offset;
    tf::quaternionMsgToTF(imu->orientation, heading);
    yaw_offset = tf::createQuaternionFromYaw(M_PI / 2);

    heading *= yaw_offset;
    tf::quaternionTFToMsg(heading, fix_trans.transform.rotation);
}
