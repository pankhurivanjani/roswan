#include "swan_controller/controller.h"
#include <exception>

Controller::Controller()
    :heading(3 * M_PI), turn(0)
{
    _heading = heading;
}
Controller::~Controller(){
    delete r, spin_thread;
}


const void Controller::run(){
    basic_setup();
    current_time = last_time = last_cmd_time = ros::Time::now().toSec();
    spin_thread = new boost::thread(boost::bind(&Controller::spinThread, this));

    setup();
    if(n.ok())
        stop();
    r = new ros::Rate(frequency);
    ros::Duration(0.5).sleep();
    try{
        while(n.ok()){
            current_time = ros::Time::now().toSec();
            update_spinparams();
            loop();
            last_time = current_time;
            failsafe();
            r->sleep();
        }
        stop();
        std::cout << "\033[0;33mController Stopped!\033[0m" << std::endl;
        spin_thread->join();
    }
    catch(std::exception& e){
        stop();
        std::cout << "\033[0;31m" << e.what() <<"\033[0m" << std::endl;
        std::cout << "\033[0;31mController Stopped!\033[0m" << std::endl;
        spin_thread->join();
    }
}



const void Controller::basic_setup(){

    /*  Configure Parameters  */
    ros::param::param<int>("~frequency", frequency, 20);
    ros::param::param<std::string>("~mode", mode, "STANDARD");
    imu_sub = n.subscribe("imu", 1, &Controller::imu_callback, this);


    if(mode != "DEBUG" && mode != "STANDARD"){
        ROS_WARN("Wrong mode type. using STANDARD as default");
        mode = "STANDARD";
    }

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
        enable_joy = false;
        enable_pid = enable_key = true;
        key_sub = n.subscribe("cmd_vel", 1, &Controller::key_callback, this);
    }
    ros::param::param<double>("~max_no_cmd_time", MAX_NO_CMD_TIME, 0.2);
    ros::param::param<double>("~stop_cmd_duration", STOP_CMD_DURATION, 0.2);
}



void Controller::joy_callback(const sensor_msgs::Joy::ConstPtr& joy){
    mtx.lock();
    _last_cmd_time = ros::Time::now().toSec();
    mtx.unlock();
}



void Controller::key_callback(const geometry_msgs::Twist::ConstPtr& msg){
    mtx.lock();
    _key_speed = msg->linear.x;
    _key_turn = msg->angular.z;
    _last_cmd_time = ros::Time::now().toSec();
    mtx.unlock();
}



void Controller::imu_callback(const sensor_msgs::Imu::ConstPtr& imu){
    mtx.lock();
    _heading = tf2::getYaw(imu->orientation);
    mtx.unlock();
}



const bool Controller::failsafe(){
    double no_cmd_time = current_time - last_cmd_time;
    if(no_cmd_time > MAX_NO_CMD_TIME){
        ROS_WARN("Failsafe activates");
        while(n.ok() && no_cmd_time > MAX_NO_CMD_TIME){
            current_time = ros::Time::now().toSec();
            update_spinparams();
            no_cmd_time = current_time - last_cmd_time;
            if(no_cmd_time < (MAX_NO_CMD_TIME + STOP_CMD_DURATION))
                stop();
            r->sleep();
            last_time = current_time;
        }
        ROS_INFO("Recieve command! Failsafe disabled.");
        return true;
    }
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

void Controller::spinThread(){
    ros::spin();
}
void Controller::update_spinparams(){
    mtx.lock();
    last_cmd_time = _last_cmd_time;
    key_speed = key_speed;
    key_turn = _key_turn;
    heading = _heading;
    mtx.unlock();
    turn = (heading - last_heading) / (current_time - last_time);
    last_heading = heading;
    if(mode == "DEBUG")
        debug_display();
}

void Controller::debug_display(){
    ;
}


