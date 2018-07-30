#include <r2c2/captain.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "ros/package.h"
namespace pt = boost::property_tree;

int main(int argc, char **argv){
    ros::init(argc, argv, "mission_test_captain");
    std::string path = ros::package::getPath("r2c2");
    std::cout <<  path << std::endl;
    pt::ptree mission_points;
    pt::read_json(path + "/params/mission_points.json", mission_points);
    std::vector<std::vector<double> > _mission_points;
    std::vector<double> coordinate;
    for(pt::ptree::value_type &row : mission_points){
        coordinate.clear();
        for(pt::ptree::value_type &cell : row.second){
            coordinate.push_back(cell.second.get_value<double>());
        }
        _mission_points.push_back(coordinate);
    }
    Captain captain;
    captain.run();
    captain.simple_point_mission(_mission_points);
    return 0;
}
