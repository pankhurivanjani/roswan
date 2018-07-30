cmake_minimum_required(VERSION 2.8.3)
project(os5000_compass)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES os5000_compass
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(os5000_compass src/os5000.cpp)
target_link_libraries(os5000_compass ${catkin_LIBRARIES})
add_dependencies(os5000_compass ${PROJECT_NAME}_gencpp)
