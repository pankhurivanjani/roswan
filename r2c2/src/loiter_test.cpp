#include <r2c2/captain.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "loiter_test_captain");
    Captain captain;
    captain.run();
    captain.loiter();
    return 0;
}
