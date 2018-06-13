#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/*
 * Author: Chen Bainian
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <swan_controller/controller.h>
#include <swan_controller/swanPIDConfig.h>
#include <swan_msgs/PID_diagnostic.h>

class PID_Controller : public Controller{
protected:
    double ki, kp, kd;
    double p_gain, i_gain, d_gain;
    double input, feedback, err;
    dynamic_reconfigure::Server<swan_controller::swanPIDConfig> server;
    dynamic_reconfigure::Server<swan_controller::swanPIDConfig>::CallbackType cb;
    ros::Publisher diag_pub;

    const double pid(){
        double new_err = input - feedback;
        p_gain = kp * new_err;
        i_gain += ki * new_err;
        d_gain = kd * (new_err - err);
        err = new_err;
        diagnostic_pub();
        return p_gain + i_gain + d_gain;
    }

public:
    PID_Controller();
    ~PID_Controller();

    const void pid_setup();
    void pid_reconfigure_callback(swan_controller::swanPIDConfig &config, uint32_t level);
    const void diagnostic_pub();
        
};

#endif
