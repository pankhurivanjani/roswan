#include "swan_sim/auto_sim_core.h"

AutoSim::AutoSim()
{
}

AutoSim::~AutoSim()
{
}

void AutoSim::publishMessage(ros::Publisher *pub_message)
{
  geometry_msgs::Twist msg;
  msg.linear.x = _linear_speed;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = _angular_speed;
  pub_message->publish(msg);
}

void AutoSim::messageCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  _linear_speed = msg->linear.x;
  _angular_speed = msg->angular.z;

  //echo linear and angular speed
  ROS_INFO("linear_speed:  %f", _linear_speed);
  ROS_INFO("angular_speed: %f", _angular_speed);
}

void AutoSim::configCallback(swan_sim::swanVelSimConfig &config, double level)
{
  //for autosim GUI
  _linear_speed = config.linear_speed;
  _angular_speed = config.angular_speed;

}
