cmake_minimum_required (VERSION 2.8.3)
project(os5000)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    sensor_msgs
    dynamic_reconfigure
)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(os5000 src/os5000_core.cpp src/os5000_ros.cpp src/serial.cpp src/timing.cpp)
target_link_libraries(os5000 ${catkin_LIBRARIES})
add_dependencies(os5000 os5000_generate_messages_cpp)

