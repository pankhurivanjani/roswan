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

	/* PID parameters */
    double ki, kp, kd;
    double p_gain, i_gain, d_gain;
    double input, feedback, err;

    /* Dynamic reconfigure parameters */
    dynamic_reconfigure::Server<swan_controller::swanPIDConfig> server;
    dynamic_reconfigure::Server<swan_controller::swanPIDConfig>::CallbackType cb;

    /* Publisher */
    ros::Publisher diag_pub;

    /* Calculate gains */
    const double pid(){
        double new_err = input - feedback;
        p_gain = kp * new_err;
        i_gain += ki * new_err;
        d_gain = kd * (new_err - err);
        err = new_err;
        double total_gain = p_gain + i_gain + d_gain;
        return total_gain;
    }

public:
    PID_Controller();
    ~PID_Controller();

    const void pid_setup();
    void pid_reconfigure_callback(swan_controller::swanPIDConfig &config, uint32_t level);
    const void diagnostic_pub();
    void debug_display();
};

#endif
