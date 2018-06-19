#include "swan_sim/auto_sim_core.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "auto_cmd_sim");
  ros::NodeHandle nh;

  AutoSim *auto_sim = new AutoSim();

  dynamic_reconfigure::Server<swan_sim::swanVelSimConfig> dr_srv;
  dynamic_reconfigure::Server<swan_sim::swanVelSimConfig>::CallbackType cb;
  cb = boost::bind(&AutoSim::configCallback, auto_sim, _1, _2);
  dr_srv.setCallback(cb);

  int rate;

  ros::NodeHandle pnh("~");
  pnh.param("rate", rate, 10);

  ros::Publisher pub_message = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::Rate r(rate);

  while (nh.ok())
  {
    auto_sim->publishMessage(&pub_message);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
