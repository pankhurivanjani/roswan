#include <swan_controller/diff_controller.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "swan_controller_node");
    int frequency;
    ros::param::param<int>("~frequency", frequency, 20);
    Diff_Controller swan_controller(frequency);
}
