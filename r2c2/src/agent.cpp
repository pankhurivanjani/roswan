#include <r2c2/agent.h>

Agent::Agent(void)
    :n(), pnh("~")
{
    ;    
}

Agent::~Agent(void)
{
    ;
}

void Agent::spinThread(){
    ros::spin();
}

bool Agent::serviceCallback(r2c2::agent_request::Request &req, r2c2::agent_request::Response &res){
    res.capability = capabilities[req.mission];
}

const void Agent::init(const std::string& _id)
{
    id = _id;
    XmlRpc::XmlRpcValue temp_params;
    if(!pnh.getParam("capabilities", temp_params)){
        ROS_ERROR("Capability list not available");
        return;
    }
    for(auto& _mission : temp_params[id]){
        capabilities[std::string(_mission.first)] = bool(_mission.second);
    }
    setup();

    check_capability_service = n.advertiseService(id + "/check_capability", &Agent::serviceCallback, this);
    ROS_INFO("Agent %s ready for mission", id.c_str());
    boost::thread spin_thread(boost::bind(&Agent::spinThread, this));

    run();

    ros::shutdown();
    spin_thread.join();
}

void run(){
    ;
}

void setup(){
    ;
}

