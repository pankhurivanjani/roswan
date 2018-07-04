#ifndef DYNAMIC_ODOM_ESTIMATOR
#define DYNAMIC_ODOM_ESTIMATOR

/*
 * Author: Chen Bainian
 * E-mail: brian97cbn@gmail.com
 * Version: 1.0
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <exception>

namespace swan_localization{

struct DynamicOdomParams{
    // Dynamic parameters
    double vx, vy, vth;
    double x, y, z, th, dt;
    bool initiated;
};


class DynamicOdomEstimator{
protected:
    ros::NodeHandle n;
    ros::NodeHandle nh;

    // Subscriber for cmd msgs
    ros::Subscriber cmd_sub, initialpose_sub, initialodom_sub;

    // Publisher for odom and tf
    ros::Publisher odom_pub;
    tf::TransformBroadcaster tf_broadcaster;

    // current time
    int frequency;
    double current_time, last_time;

    // frame ids
    std::string frame_id, child_frame_id;

    // ros param
    bool send_transform, sim_mode, holonomic;

    DynamicOdomParams dynamic_params;

    const void init_DynamicOdomParams(DynamicOdomParams& _dynamic_params){
        _dynamic_params.vx = _dynamic_params.vy = _dynamic_params.vth = 0;
        _dynamic_params.x = _dynamic_params.y = _dynamic_params.z = _dynamic_params.th = _dynamic_params.dt = 0;
        _dynamic_params.initiated = false;
    }

public:
    DynamicOdomEstimator();
    ~DynamicOdomEstimator();
    void cmd_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void initialodom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    const void run();
    const void setup();
    virtual void loop();
    const void update_position(DynamicOdomParams& _dynamic_params);
    const void pub_odom() const;
    const void pub_tf();
};

}
#endif
