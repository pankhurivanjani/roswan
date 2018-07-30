#ifndef SW_AGENT_H
#define SW_AGENT_H

#include <ros/ros.h>
#include <r2c2/agent_request.h>
#include <boost/thread.hpp>
#include <vector>
#include <map>

class Agent
{
protected:
    std::string id;
    std::map<std::string, bool> capabilities;
    ros::NodeHandle n;
    ros::NodeHandle pnh;
    ros::ServiceServer check_capability_service;

public:
    Agent();
    ~Agent();
    const void init(const std::string& _id);
    virtual void run(){};
    virtual void setup(){};
    void spinThread();
    bool serviceCallback(r2c2::agent_request::Request &req, r2c2::agent_request::Response &res);
};

#endif
