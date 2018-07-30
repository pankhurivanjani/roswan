/*
 * Author: Chen Bainian
 * E-mail: brian97cbn@gmail.com
 * Version: 1.0
 *
 */

#include <r2c2/captain.h>

Captain::Captain()
    :nh("~"), ac("move_base", true)
{
    odom_sub = n.subscribe("odom", 1, &Captain::odom_callback, this);
}

Captain::~Captain()
{
    ;
}

const void Captain::run(){
    ac.waitForServer();
    ROS_INFO("Action server started.");
}


const void Captain::simple_point_mission(const std::vector<std::vector<double> >& mission_points){
    move_base_msgs::MoveBaseGoal goal;
    for(int i = 0; i < mission_points.size(); ++i){

    	// Pass the mission point coordinates to goal msg
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.position.x = mission_points[i][0];
        goal.target_pose.pose.position.y = mission_points[i][1];
        goal.target_pose.pose.position.z = 0;
        if(i < mission_points.size() - 1){
        	double dx = mission_points[i + 1][0] - mission_points[i][0];
        	double dy = mission_points[i + 1][1] - mission_points[i][1];
            tf2::Quaternion q;
            q.setRPY(0, 0, atan2(dy, dx));
            tf2::convert(q, goal.target_pose.pose.orientation);
        }

        ac.sendGoal(goal);

        ROS_INFO("Goal: (%.2f, %.2f) sent", mission_points[i][0], mission_points[i][1]);

        //Wait for the result
        if(ac.waitForResult())
            ROS_INFO("Action finished: %s", ac.getState().toString().c_str());
        else{
            ROS_WARN("Action did not finish before the time out.");
            continue;
        }
    }
}


const void Captain::loiter(){

	// Capture the loitering point
    ROS_INFO("Waiting for valid odom.");
    while(odom.pose.pose.orientation.w == 0){
        ros::spinOnce();
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose = odom.pose.pose;
    ROS_INFO("Goal: (%.2f, %.2f) sent", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    tf2::Transform loiter_pose;
    tf2::convert(goal.target_pose.pose, loiter_pose);
    ROS_INFO("Start loitering...");

    while(n.ok()){
        ros::spinOnce();
        tf2::Transform current_pose;
        tf2::convert(odom.pose.pose, current_pose);
        double distance_from_goal = tf2::tf2Distance(loiter_pose.getOrigin(), current_pose.getOrigin());
        ROS_DEBUG("Distance from goal: %.2f", distance_from_goal);
        // Keep sending the loitering point when robot is out of the loiter range
        if(distance_from_goal >= 5){
            ROS_INFO("Out of loiter range by %.2f, traveling back...", distance_from_goal);
            ac.sendGoal(goal);
            if(ac.waitForResult()){
                ROS_INFO("Result: %s", ac.getState().toString().c_str());
            }
        }
        ros::Duration(0.5).sleep();

    }

}

// Update the robot location
void Captain::odom_callback(const nav_msgs::OdometryConstPtr& msg){
    odom.pose.pose.position.x = msg->pose.pose.position.x;
    odom.pose.pose.position.y = msg->pose.pose.position.y;
    odom.pose.pose.position.z = msg->pose.pose.position.z;
    odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;
}
