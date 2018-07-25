cmake_minimum_required(VERSION 2.8.3)
project(razor_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES razor_imu
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(razor_imu src/razor_imu.cpp)
target_link_libraries(razor_imu ${catkin_LIBRARIES})
add_dependencies(razor_imu ${PROJECT_NAME}_gencpp)
