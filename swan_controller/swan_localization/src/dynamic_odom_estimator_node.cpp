#include <swan_localization/dynamic_odom_estimator.h>
using namespace swan_localization;

int main(int argc, char** argv){
    ros::init(argc, argv, "dynamic_odom_estimator");
    DynamicOdomEstimator dynamic_odom_estimator;
    dynamic_odom_estimator.run();
}
