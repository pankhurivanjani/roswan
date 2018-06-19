#ifndef AUTO_SIM_CORE_H
#define AUTO_SIM_CORE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <swan_sim/swanVelSimConfig.h>

class AutoSim
{
public:
  AutoSim();
  ~AutoSim();
  void configCallback(swan_sim::swanVelSimConfig &config, double level);
  void publishMessage(ros::Publisher *pub_message);
  void messageCallback(const geometry_msgs::Twist::ConstPtr &msg);

  double _linear_speed;
  double _angular_speed;
};
#endif
