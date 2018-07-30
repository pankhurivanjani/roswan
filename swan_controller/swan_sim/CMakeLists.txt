cmake_minimum_required(VERSION 2.8.3)
project(swan_sim)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
)
generate_dynamic_reconfigure_options(
    cfg/swanVelSim.cfg
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(constant_speed_sim src/constant_speed_sim.cpp)
target_link_libraries(constant_speed_sim ${catkin_LIBRARIES})
add_dependencies(constant_speed_sim swan_sim_generate_messages_cpp)

add_executable(auto_sim src/auto_sim.cpp src/auto_sim_core.cpp)
target_link_libraries(auto_sim ${catkin_LIBRARIES})
add_dependencies(auto_sim ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
