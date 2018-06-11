#ifndef DIFF_CONTROLLER_H
#define DIFF_CONTROLLER_H

/*
 * Author: Chen Bainian
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <swan_controller/pid_controller.h>

class Diff_Controller : public PID_Controller{
protected:
    double desired_speed, desired_turn, desired_heading;
    double last_heading, last_turn;
    double cmd_speed, cmd_turn;
    double pwr_min, pwr_max, speed_min, speed_max;
    std::string type;

    ros::Publisher l_pub, r_pub;

public:
    Diff_Controller();
    // Main Constructor
    explicit Diff_Controller(const int frequency);

    void setup();
    void desired_heading_estimator();
    void last_turn_estimator();
    void loop();
    void stop();
    void diff_drive(const double speed, const double turn);
};

#endif
