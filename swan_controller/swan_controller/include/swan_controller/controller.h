#ifndef SW_CONTROLLER_H
#define SW_CONTROLLER_H

/*
 * Author: Chen Bainian
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <string>

#include <gps_common/conversions.h>


class Controller{
protected:
    ros::NodeHandle n;
    ros::Rate r;



    /*  Publisher and Subscriber  */
    // Subscriber for joy and keyboard testing
    ros::Subscriber joy_sub, key_sub;

    // Subscriber for autonomous cmd;
    ros::Subscriber auto_sub;

    // Publisher to the motor driver;
    ros::Publisher l_pub, r_pub;
    
    // Subscriber for feedback from IMU and GPS
    ros::Subscriber imu_sub, fix_sub;

    // Odom and TF publisher
    ros::Publisher odom_pub;
    tf::TransformBroadcaster tf_broadcaster;




    /*  Command and Configuration */
    // running mode
    std::string mode;
    bool enable_joy, enable_key, enable_pid;
    // linear and angular speed to motor driver
    double cmd_speed, cmd_turn;

    // desired and feedback displacement and yaw;
    double desired_displacement, desired_yaw;
    double fb_displacement, fb_yaw;

    // PID parameters
    double kp, ki, kd;

    // TF and odom config
    bool publish_transform, publish_odom;
    std::string frame_id, child_frame_id;


    
    // Failsafe parameters
    double MAX_NO_CMD_TIME, STOP_CMD_DURATION;
    double current_time, last_time, last_cmd_time;

    const inline double constrain(double a, double min, double max){
        if(a > max)
            return max;
        else if(a < - max)
            return -max;
        else if(a < min && a > - min)
            return 0;
        else
            return a;
    }

public:
    Controller();

    // Main Constructor
    explicit Controller(const int frequency);
    virtual void setup();
    virtual void loop();
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
    void key_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& fix);
    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu);
    bool failsafe();
    const void stop();
    virtual void diff_drive();


};

#endif
