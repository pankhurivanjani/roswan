#include <swan_controller/pid_controller.h>

PID_Controller::PID_Controller()            
{
    ;
}

PID_Controller::PID_Controller(const int frequency)
{
    run();
}

const void PID_Controller::pid_setup(){
    if(mode == "DEBUG"){
        if(enable_pid){
            ros::param::param<double>("~kp", kp, 1);
            ros::param::param<double>("~ki", ki, 0);
            ros::param::param<double>("~kd", kd, 0);
        }
    }
    else if(mode == "STANDARD"){
        ros::param::param<double>("~kp", kp, 1);
        ros::param::param<double>("~ki", ki, 0);
        ros::param::param<double>("~kd", kd, 0);
    }
    err = input = output = p_gain = i_gain = d_gain = 0;
}
