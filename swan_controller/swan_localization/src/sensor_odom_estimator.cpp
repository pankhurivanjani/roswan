/*
 * Author: Chen Bainian
 */
#include <swan_localization/utm_tf_node.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "sensor_odom_estimator");
    UTM_tf_node utm_tf_node;
    utm_tf_node.init();
}
