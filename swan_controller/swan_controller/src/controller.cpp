#include "swan_controller/controller.h"
#include <exception>

Controller::Controller()
    :heading(0.5)
{
    ;
}
Controller::~Controller(){
    ;
}


const void Controller::run(){
    basic_setup();
    setup();
    if(n.ok())
        stop();
    ros::Rate r(frequency);
    ros::Duration(0.5).sleep();
    current_time = last_time = last_cmd_time = ros::Time::now().toSec();
    try{
        while(n.ok()){
            ros::spinOnce();
            current_time = ros::Time::now().toSec();
            loop();
            last_time = current_time;
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



const void Controller::basic_setup(){

    /*  Configure Parameters  */
    ros::param::param<int>("~frequency", frequency, 20);
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
        ros::param::param<bool>("~enable_pid", enable_pid, true);
    }
    else if(mode == "STANDARD"){
        ;
    }
    else{
        ROS_WARN("Wrong mode type. using STANDARD as default");
        mode = "STANDARD";
    }
    ros::param::param<double>("~max_no_cmd_time", MAX_NO_CMD_TIME, 0.2);
    ros::param::param<double>("~stop_cmd_duration", STOP_CMD_DURATION, 0.2);
}



void Controller::joy_callback(const sensor_msgs::Joy::ConstPtr& joy){
    ;
    last_cmd_time = ros::Time::now().toSec();
}



void Controller::key_callback(const geometry_msgs::Twist::ConstPtr& msg){
    key_speed = msg->linear.x;
    key_turn = msg->angular.z;
    last_cmd_time = ros::Time::now().toSec();
}



void Controller::imu_callback(const sensor_msgs::Imu::ConstPtr& imu){
    heading = tf::getYaw(imu->orientation);
}



const bool Controller::failsafe(){
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



void Controller::loop(){
    ;    
}



void Controller::setup(){
    basic_setup();
}



void Controller::stop(){
    ;
}

