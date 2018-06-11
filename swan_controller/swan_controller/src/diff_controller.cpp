#include <swan_controller/diff_controller.h>

Diff_Controller::Diff_Controller(){
    ;
}

Diff_Controller::Diff_Controller(const int frequency){
    run();
}

void Diff_Controller::setup(){
    if(enable_pid)
        pid_setup();
    last_turn = 0;
    desired_speed = desired_turn = desired_heading = last_heading = 0;
    input = output = 0;
    ros::param::param<std::string>("~type", type, "TURN" );
    ros::param::param<double>("~pwr_min", pwr_min, 0.4);
    ros::param::param<double>("~pwr_max", pwr_max, 0.9);
    ros::param::param<double>("~speed_min", speed_min, 0);
    ros::param::param<double>("~speed_max", speed_max, 2);

    l_pub = n.advertise<std_msgs::Float64>("l_motor", 10);
    r_pub = n.advertise<std_msgs::Float64>("r_motor", 10);

    while(n.ok()){
        ros::spinOnce();
        if(heading >= 0 && heading <= 2 * M_PI){
            break;
        }
        ROS_WARN("Heading out of range, heading: %f", heading);
        r.sleep();
    }
    desired_heading = last_heading = heading;
}

void Diff_Controller::desired_heading_estimator(){
    double dt = current_time - last_time;
    desired_heading += desired_turn * dt;
    while(std::abs(desired_heading - heading) >= 2 * M_PI){
        if((desired_heading - heading) >= 2 * M_PI){
            desired_heading -= 2 * M_PI;
        }
        else if((desired_heading - heading) <= - 2 * M_PI){
            desired_heading += 2 * M_PI;
        }
    } 
}

void Diff_Controller::last_turn_estimator(){
    double dt = current_time - last_time;
    last_turn = (heading - last_heading) / dt;
    last_heading = heading;
}

void Diff_Controller::loop(){
    if(!failsafe()){
        if(enable_key){
            if(key_speed != 0 || key_turn != 0){
                desired_speed = key_speed;
                desired_turn = key_turn;
            }
            else if(enable_joy){
                if(false){
                    ;
                }
                else{
                    desired_speed = auto_speed;
                    desired_turn = auto_turn;
                }
            }
        }

        if(type == "TURN"){
            last_turn_estimator();
            input = desired_turn;
            output = last_turn;
        }
        else if(type == "HEADING"){
            desired_heading_estimator();
            input = desired_heading;
            output = heading;
        }
        cmd_speed = desired_speed;
        cmd_turn = pid();
        ROS_INFO("desired_heading = %.5f", desired_heading);
        diff_drive(cmd_speed, cmd_turn);
    }
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
}
