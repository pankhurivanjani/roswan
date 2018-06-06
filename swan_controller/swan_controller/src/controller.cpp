#include "swan_controller/controller.h"
#include <exception>

Controller::Controller()
    :r(0)
{
    ;
}

Controller::Controller(const int frequency)
    :r(frequency), cmd_speed(0), cmd_turn(0)
{
    setup();
    ros::Duration(0.5).sleep();
    current_time = last_time = last_cmd_time = ros::Time::now().toSec();
    try{
        while(n.ok()){
            ros::spinOnce();
            current_time = ros::Time::now().toSec();
            loop();
            r.sleep();
        }
        stop();
        std::cout << "\033[0;33mController Stopped!\033[0m" << std::endl;
    }
    catch(std::exception& e){
        stop();
        std::cout << "\033[0;31m" << e.what() <<"\033[0m" << std::endl;
        std::cout << "\033[0;31mController Stopped!\033[0m" << std::endl;
    }
}

void Controller::setup(){

    /*  Configure Parameters  */
    ros::param::param<std::string>("~mode", mode, "STANDARD");

    if(mode == "DEBUG"){
        ros::param::param<bool>("~enable_joy", enable_joy, false);
        if(enable_joy){
            joy_sub = n.subscribe("joy", 1, &Controller::joy_callback, this);
        }
        ros::param::param<bool>("~enable_key", enable_key, true);
        if(enable_key){
            key_sub = n.subscribe("cmd_vel", 1, &Controller::key_callback, this);
        }
        ros::param::param<bool>("~enable_pid", enable_key, true);
        if(enable_pid){
            //Dynamic reconfiguration enable pid;
        }
    }
    else if(mode == "STANDARD"){
        ros::param::param<double>("~kp", kp, 1);
        ros::param::param<double>("~ki", ki, 0);
        ros::param::param<double>("~kd", kd, 0);
    }
    else{
        ROS_WARN("Wrong mode type. using STANDARD as default");
        mode = "STANDARD";
    }
    ros::param::param<double>("~max_no_cmd_time", MAX_NO_CMD_TIME, 0.2);
    ros::param::param<double>("~stop_cmd_duration", STOP_CMD_DURATION, 0.2);
    
    l_pub = n.advertise<std_msgs::Float64>("l_motor", 1);
    r_pub = n.advertise<std_msgs::Float64>("r_motor", 1);
}


void Controller::loop(){
    if(!failsafe())
        diff_drive();
}

void Controller::key_callback(const geometry_msgs::Twist::ConstPtr& msg){
    cmd_speed = msg->linear.x;
    cmd_turn = msg->angular.z;
    last_cmd_time = ros::Time::now().toSec();
}


void Controller::joy_callback(const sensor_msgs::Joy::ConstPtr& joy){
    ;
}

void Controller::diff_drive(){
    std_msgs::Float64 l_msg, r_msg;
    double raw_l_pwr, raw_r_pwr, l_pwr, r_pwr;
    raw_l_pwr = cmd_speed - cmd_turn;
    raw_r_pwr = cmd_speed + cmd_turn;
    l_pwr = constrain(raw_l_pwr, 0.4, 0.9);
    r_pwr = constrain(raw_r_pwr, 0.4, 0.9);
    l_msg.data = l_pwr;
    r_msg.data = r_pwr;
    l_pub.publish(l_msg);
    r_pub.publish(r_msg);
}

const void Controller::stop(){
    cmd_speed = 0;
    cmd_turn = 0;
    diff_drive();
}

bool Controller::failsafe(){
    double no_cmd_time = current_time - last_cmd_time;
    if(no_cmd_time > MAX_NO_CMD_TIME && no_cmd_time < (MAX_NO_CMD_TIME + STOP_CMD_DURATION)){
        stop();
        return true;
    }
    else if(no_cmd_time >= (MAX_NO_CMD_TIME + STOP_CMD_DURATION))
        return true;
    else
        return false;
}
