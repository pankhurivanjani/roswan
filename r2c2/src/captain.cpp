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
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.position.x = mission_points[i][0];
        goal.target_pose.pose.position.y = mission_points[i][1];
        goal.target_pose.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        tf2::convert(q, goal.target_pose.pose.orientation);

        ac.sendGoal(goal);

        ROS_INFO("Goal: (%.2f, %.2f) sent", mission_points[i][0], mission_points[i][1]);
        if(ac.waitForResult())
            ROS_INFO("Action finished: %s", ac.getState().toString().c_str());
        else{
            ROS_WARN("Action did not finish before the time out.");
            continue;
        }
    }
}


const void Captain::loiter(){
    ros::spinOnce();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose = odom.pose.pose;
    tf2::Transform loiter_pose;
    tf2::convert(goal.target_pose.pose, loiter_pose);
    ROS_INFO("Start loitering...");
    while(true){
        ros::spinOnce();
        tf2::Transform current_pose;
        tf2::convert(odom.pose.pose, current_pose);
        double distance_from_goal = tf2::tf2Distance(loiter_pose.getOrigin(), current_pose.getOrigin());
        ROS_INFO("%.2f", distance_from_goal);
        if(distance_from_goal >= 5){
            ROS_INFO("Out of loiter range, traveling back...");
            ac.sendGoal(goal);
            if(ac.waitForResult()){
                ROS_INFO("Back to position");
            }
        }
        ros::Duration(0.5).sleep();
        ROS_INFO("a");

    }

}

void Captain::odom_callback(const nav_msgs::OdometryConstPtr& msg){
    odom = *msg;
}
