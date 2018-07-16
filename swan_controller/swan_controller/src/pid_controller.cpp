#include <swan_controller/pid_controller.h>

PID_Controller::PID_Controller()            
{
    ;
}

PID_Controller::~PID_Controller()
{
    ;
}

const void PID_Controller::pid_setup(){
    input = feedback = 0;
    if(mode == "DEBUG"){
        if(enable_pid){
            ros::param::param<double>("~kp", kp, 1);
            ros::param::param<double>("~ki", ki, 0);
            ros::param::param<double>("~kd", kd, 0);
            cb = boost::bind(&PID_Controller::pid_reconfigure_callback, this, _1, _2);
            server.setCallback(cb);
            diag_pub = n.advertise<swan_msgs::PID_diagnostic>("pid_diagnostic", 10); 
        }
    }
    else if(mode == "STANDARD"){
        ros::param::param<double>("~kp", kp, 1);
        ros::param::param<double>("~ki", ki, 0);
        ros::param::param<double>("~kd", kd, 0);
        ROS_INFO("Initialize with PID Constants: kp=%.2f,\tki=%.2f,\tkd=%.2f.", kp, ki, kd);
    }
    err = input = feedback = p_gain = i_gain = d_gain = 0;
}

void PID_Controller::pid_reconfigure_callback(swan_controller::swanPIDConfig &config, uint32_t level){
    kp = config.p * pow(10, config.p_scale - 2);
    ki = config.i * pow(10, config.i_scale - 2);
    kd = config.d * pow(10, config.d_scale - 2);
    ROS_INFO("Reconfigure PID Constant: kp=%.2f,\tki=%.2f,\tkd=%.2f.", kp, ki, kd);
}

const void PID_Controller::diagnostic_pub(){
    swan_msgs::PID_diagnostic diagnostic_msg;
    diagnostic_msg.header.stamp = ros::Time::now();
    diagnostic_msg.p = kp;
    diagnostic_msg.i = ki;
    diagnostic_msg.d = kd;
    diagnostic_msg.p_gain = p_gain;
    diagnostic_msg.i_gain = i_gain;
    diagnostic_msg.d_gain = d_gain;
    diagnostic_msg.input = input;
    diagnostic_msg.feedback = feedback;
    diagnostic_msg.loop_error = err;
    diagnostic_msg.gain = p_gain + i_gain + d_gain;
    diagnostic_msg.position = position;
    diagnostic_msg.heading = heading;
    diagnostic_msg.turn = turn;
    diag_pub.publish(diagnostic_msg);
}
