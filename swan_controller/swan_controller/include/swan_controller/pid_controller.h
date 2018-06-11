#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/*
 * Author: Chen Bainian
 */

#include <ros/ros.h>

#include <swan_controller/controller.h>

class PID_Controller : public Controller{
protected:
    double ki, kp, kd;
    double p_gain, i_gain, d_gain;
    double input, output, err;

    const double pid(){
        double new_err = input - output;
        p_gain = kp * new_err;
        i_gain += ki * new_err;
        d_gain = kd * (new_err - err);
        err = new_err;
        return p_gain + i_gain + d_gain;
    }

public:
    PID_Controller();

    explicit PID_Controller(const int frequency);
    const void pid_setup();
        
};

#endif
