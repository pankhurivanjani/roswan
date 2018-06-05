#include <gps_tools/utm_tf_node.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "utm_tf_node");
    UTM_tf_node utm_tf_node(1);
}
