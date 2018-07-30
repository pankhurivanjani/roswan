cmake_minimum_required(VERSION 2.8.3)
project(swan_controller)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  swan_msgs
  tf2
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
    cfg/swanPID.cfg
)

catkin_package()
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/swan_controller_node.cpp src/diff_controller.cpp src/pid_controller.cpp src/controller.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)