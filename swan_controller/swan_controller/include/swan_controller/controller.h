#ifndef SW_CONTROLLER_H
#define SW_CONTROLLER_H

/*
 * Author: Chen Bainian
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <string>


class Controller{
protected:
    ros::NodeHandle n;
    ros::Rate r;



    /*  Publisher and Subscriber  */
    // Subscriber for joy and keyboard testing
    ros::Subscriber joy_sub, key_sub;

    // Subscriber for feedback from IMU
    ros::Subscriber imu_sub;


    /*  Command and Configuration */
    // running mode
    std::string mode;
    bool enable_joy, enable_key, enable_pid;
    // linear and angular speed to motor driver
    double key_speed, key_turn;
    sensor_msgs::Joy joy_cmd;
    double auto_speed, auto_turn;

    // displacement and yaw;
    double position, heading;

    
    // Failsafe parameters
    double MAX_NO_CMD_TIME, STOP_CMD_DURATION;
    double current_time, last_time, last_cmd_time;

    const inline double remap_constrain(double a, double min, double max, double map_min, double map_max){
        min = std::abs(min);
        max = std::abs(max);
        map_min = std::abs(map_min);
        map_max = std::abs(map_max);

        if(min > max)
            std::swap(min, max);
        if(map_min > map_max)
            std::swap(map_min, map_max);

        if(a > max){
            ROS_WARN("Max speed is reached");
            return map_max;
        }else if(a < - max){
            ROS_WARN("Max speed is reached");
            return -map_max;
        }else if(a <= min && a >= - min)
            return 0;
        else if(a > min && a <= max)
            return map_min + a * (map_max - map_min) / (max - min);
        else if(a < - min && a >= - max)
            return - map_min + a * (map_max - map_min) / (max - min);
        else 
            return 0;
    }
    

public:
    Controller();

    // Main Constructor
    explicit Controller(const int frequency);
    const void run();
    const void basic_setup();
    void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);
    void key_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu);
    const bool failsafe();
    virtual void setup();
    virtual void loop();
    virtual void stop();


};

#endif
