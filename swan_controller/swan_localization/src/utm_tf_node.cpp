#include <swan_localization/utm_tf_node.h>
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
    ros::param::param<bool>("~use_map_frame", use_map_frame, false);
    ros::param::param<double>("~x", map_easting, 0);
    ros::param::param<double>("~y", map_northing, 0);
    ros::Rate r(frequency);


    fix_sub = n.subscribe("fix", 1, &UTM_tf_node::update_UTM, this);
    if(use_imu_orientation)
        imu_sub = n.subscribe("imu", 1, &UTM_tf_node::update_orientation, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    
    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
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
        
        if(use_map_frame){
            reframed_coordinate_to_map();
        }
        
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
    fix_trans.transform.translation.x = odom.pose.pose.position.x;
    fix_trans.transform.translation.y = odom.pose.pose.position.y;
    fix_trans.transform.translation.z = odom.pose.pose.position.z;
    fix_trans.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster.sendTransform(fix_trans);

    if(use_map_frame){
        geometry_msgs::TransformStamped map_trans;
        map_trans.header.stamp = ros::Time::now();
        map_trans.header.frame_id = "utm";
        map_trans.child_frame_id = "map";
        map_trans.transform.translation.x = map_easting;
        map_trans.transform.translation.y = map_northing;
        map_trans.transform.translation.z = 0;
        map_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        tf_broadcaster.sendTransform(map_trans);
    }
}

void UTM_tf_node::update_orientation(const sensor_msgs::Imu::ConstPtr& imu){
    tf::Quaternion heading, yaw_offset;
    tf::quaternionMsgToTF(imu->orientation, heading);
    yaw_offset = tf::createQuaternionFromYaw(M_PI / 2);

    heading *= yaw_offset;
    tf::quaternionTFToMsg(heading, odom.pose.pose.orientation);
}

void UTM_tf_node::reframed_coordinate_to_map(){
    tf::Transform utm_to_base;
    tf::poseMsgToTF(odom.pose.pose, utm_to_base);
    tf::Transform utm_to_map(tf::Quaternion(0, 0, 0, 1), tf::Vector3(map_easting, map_northing, 0));
    tf::Transform map_to_base = utm_to_map.inverse() * utm_to_base;
    tf::poseTFToMsg(map_to_base, odom.pose.pose);
}
