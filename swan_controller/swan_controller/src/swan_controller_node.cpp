#include <swan_controller/diff_controller.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "swan_controller_node");
    Diff_Controller swan_controller;
    std::cout << "controller initiated\n";
    swan_controller.run();
}
