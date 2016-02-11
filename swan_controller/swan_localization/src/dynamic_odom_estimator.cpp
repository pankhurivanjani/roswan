/*
 * Author: Chen Bainian
 * Email: brian97cbn@gmail.com
 * Version: 1.0
 *
 */

#include <swan_localization/dynamic_odom_estimator.h>

using namespace swan_localization;

DynamicOdomEstimator::DynamicOdomEstimator()
    :nh("~")
{
    init_DynamicOdomParams(dynamic_params);
}

DynamicOdomEstimator::~DynamicOdomEstimator(){
    ;
}

const void DynamicOdomEstimator::run(){
    setup();
    ros::Rate r(frequency);
    ros::Duration(0.5).sleep();
    current_time = last_time = last_cmd_time = ros::Time::now().toSec();
    try{
        while(n.ok()){
            ros::spinOnce();
            current_time = ros::Time::now().toSec();
            if(failsafe_estimate())
                loop();
            last_time = current_time;

            r.sleep();
        }
    }
    catch(std::exception& e){
        std::cout << "\033[0;31m" << e.what() << "\033[0m\n";
    }
}

const void DynamicOdomEstimator::setup(){
    nh.param("frequency", frequency, int(20));
    nh.param("frame_id", frame_id, std::string("odom"));
    nh.param("child_frame_id", child_frame_id, std::string("base_link"));
    nh.param("send_transform", send_transform, bool(false));
    nh.param("sim_mode", sim_mode, bool(true));
    nh.param("holonomic", holonomic, bool(false));
    nh.param("max_no_cmd_time", MAX_NO_CMD_TIME, 0.5);
    nh.param("stop_cmd_duration", STOP_CMD_DURATION, 0.2);


    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    cmd_sub = n.subscribe("cmd_vel", 1, &DynamicOdomEstimator::cmd_callback, this);


    if(sim_mode){
        initialpose_sub = n.subscribe("initialpose", 1, &DynamicOdomEstimator::initialpose_callback, this);
        std::cout << "Sim mode!\nWaiting for initial pose...\n";
        while(n.ok()){
            ros::Duration(1 / (double)frequency).sleep();
            ros::spinOnce();
            if(dynamic_params.initiated){
                ROS_INFO("initial pose recieved!");
                break;
            }
        }
    }
    else{
        initialodom_sub = n.subscribe("initialodom", 1, &DynamicOdomEstimator::initialodom_callback, this);
        std::cout << "Normal mode!\nWaiting for initial odom from gps...\n";
        while(n.ok()){
            ros::Duration(1 / (double)frequency).sleep();
            ros::spinOnce();
            if(dynamic_params.initiated){
                ROS_INFO("initial gps odom recieved!");
                break;
            }
        }
    }

}

void DynamicOdomEstimator::loop(){
    dynamic_params.dt = current_time - last_time;
    update_position(dynamic_params);
    pub_odom();
    if(send_transform)
        pub_tf();
}

const void DynamicOdomEstimator::update_position(DynamicOdomParams& _dynamic_params){
    double dx = dynamic_params.vx * cos(dynamic_params.th) * dynamic_params.dt + dynamic_params.vy * sin(dynamic_params.th) * dynamic_params.dt;
    double dy = dynamic_params.vx * sin(dynamic_params.th) * dynamic_params.dt - dynamic_params.vy * cos(dynamic_params.th) * dynamic_params.dt;
    dynamic_params.x += dx;
    dynamic_params.y += dy;
    dynamic_params.th += dynamic_params.vth * dynamic_params.dt;
}

void DynamicOdomEstimator::cmd_callback(const geometry_msgs::Twist::ConstPtr& msg){
    dynamic_params.vx = msg->linear.x;
    dynamic_params.vy = (holonomic) ? msg->linear.y : 0;
    dynamic_params.vth = msg->angular.z;
    last_cmd_time = ros::Time::now().toSec();
}

void DynamicOdomEstimator::initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    dynamic_params.x = msg->pose.pose.position.x;
    dynamic_params.y = msg->pose.pose.position.y;
    dynamic_params.z = msg->pose.pose.position.z;
    dynamic_params.th = tf2::getYaw(msg->pose.pose.orientation);
    dynamic_params.initiated = true;
}

void DynamicOdomEstimator::initialodom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    if(!dynamic_params.initiated){
        dynamic_params.x = msg->pose.pose.position.x;
        dynamic_params.y = msg->pose.pose.position.y;
        dynamic_params.z = msg->pose.pose.position.z;
        dynamic_params.th = tf2::getYaw(msg->pose.pose.orientation);
        dynamic_params.initiated = true;
    }
}

const void DynamicOdomEstimator::pub_odom() const{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;
    odom.pose.pose.position.x = dynamic_params.x;
    odom.pose.pose.position.y = dynamic_params.y;
    odom.pose.pose.position.z = dynamic_params.z;
    tf2::Quaternion qt;
    qt.setRPY(0,0, dynamic_params.th);
    tf2::convert(qt, odom.pose.pose.orientation);
    odom.twist.twist.linear.x = dynamic_params.vx;
    odom.twist.twist.linear.y = dynamic_params.vy;
    odom.twist.twist.angular.z = dynamic_params.vth;
    odom_pub.publish(odom);
}


const void DynamicOdomEstimator::pub_tf(){
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = frame_id;
    odom_trans.child_frame_id = child_frame_id;
    odom_trans.transform.translation.x = dynamic_params.x;
    odom_trans.transform.translation.y = dynamic_params.y;
    odom_trans.transform.translation.z = dynamic_params.z;
    tf2::Quaternion qt;
    qt.setRPY(0,0, dynamic_params.th);
    tf2::convert(qt, odom_trans.transform.rotation);
    tf2_broadcaster.sendTransform(odom_trans);
}


const bool DynamicOdomEstimator::failsafe_estimate(){
    double no_cmd_time = current_time - last_cmd_time;
    if(no_cmd_time > MAX_NO_CMD_TIME && no_cmd_time < (MAX_NO_CMD_TIME + STOP_CMD_DURATION)){
        dynamic_params.vx = 0;
        dynamic_params.vy = 0;
        dynamic_params.vth = 0;
        return true;
    }
    else{
        return false;
    }
}

