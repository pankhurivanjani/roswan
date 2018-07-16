#include <swan_controller/diff_controller.h>

Diff_Controller::Diff_Controller(){
    ;
}

Diff_Controller::~Diff_Controller(){
}

void Diff_Controller::setup(){
    if(enable_pid)
        pid_setup();
    last_turn = 0;
    desired_speed = desired_turn = desired_heading = 0;
    ros::param::param<std::string>("~type", type, "TURN" );
    ros::param::param<double>("~pwr_min", pwr_min, 0.4);
    ros::param::param<double>("~pwr_max", pwr_max, 0.9);
    ros::param::param<double>("~speed_min", speed_min, 0);
    ros::param::param<double>("~speed_max", speed_max, 2);
    ros::param::param<double>("~gain_min", min_gain, 0.05);
    ROS_INFO("Waiting for compass value");
    while(n.ok()){
        ros::spinOnce();
        if(heading >= -M_PI && heading <= M_PI){
            ROS_INFO("Compass value recieved!");
            break;
        }
        ros::Rate(frequency).sleep();
    }
    desired_heading = heading;
    r_pub = n.advertise<std_msgs::Float64>("/r_motor", 1);
    l_pub = n.advertise<std_msgs::Float64>("/l_motor", 1);

}

void Diff_Controller::desired_heading_estimator(){
    double dt = current_time - last_time;
    desired_heading += desired_turn * dt;
    while(std::abs(desired_heading - heading) >= M_PI){
        if((desired_heading - heading) >= M_PI){
            desired_heading -= 2 * M_PI;
        }
        else if((desired_heading - heading) <= - M_PI){
            desired_heading += 2 * M_PI;
        }
    } 
}

void Diff_Controller::last_turn_estimator(){
    last_turn = turn;
}

void Diff_Controller::loop(){
    if(!failsafe()){
        if(enable_joy){
            ;
        }
        else if(enable_key){
                desired_speed = key_speed;
                desired_turn = key_turn;
        }

        if(type == "TURN"){
            last_turn_estimator();
            input = desired_turn;
            feedback = last_turn;
        }
        else if(type == "HEADING"){
            desired_heading_estimator();
            input = desired_heading;
            feedback = heading;
        }
        cmd_speed = desired_speed;
        cmd_turn =  pid();
        cmd_turn = (std::abs(cmd_turn) > min_gain) ? cmd_turn : 0;
        //ROS_INFO("desired_heading = %.5f", desired_heading);
        diff_drive(cmd_speed, cmd_turn);
    }
    if(mode == "DEBUG")
        diagnostic_pub();
}

void Diff_Controller::diff_drive(const double _speed, const double _turn){
    std_msgs::Float64 l_msg, r_msg;
    double raw_l_pwr, raw_r_pwr, l_pwr, r_pwr;
    raw_l_pwr = _speed - _turn / 2.0;
    raw_r_pwr = _speed + _turn / 2.0;

    l_pwr = remap_constrain(raw_l_pwr, speed_min, speed_max, pwr_min, pwr_max);
    r_pwr = remap_constrain(raw_r_pwr, speed_min, speed_max, pwr_min, pwr_max);
    l_msg.data = l_pwr;
    r_msg.data = r_pwr;
    l_pub.publish(l_msg);
    r_pub.publish(r_msg);
}

void Diff_Controller::stop(){
    diff_drive(0, 0);
    key_speed = key_turn = joy_speed = joy_turn = auto_speed = auto_turn = desired_speed = desired_turn = 0;
}
