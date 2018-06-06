#include <swan_controller/controller.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "swan_controller_node");
    int frequency;
    ros::param::param<int>("~frequency", frequency, 20);
    Controller swan_controller(frequency);
}
