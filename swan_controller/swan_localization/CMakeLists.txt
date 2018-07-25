cmake_minimum_required(VERSION 2.8.3)
project(swan_localization)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  gps_common
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensor_odom_estimator src/sensor_odom_estimator.cpp src/utm_tf_node.cpp)
target_link_libraries(sensor_odom_estimator ${catkin_LIBRARIES})
add_dependencies(sensor_odom_estimator ${PROJECT_NAME}_generate_messages_cpp)

add_executable(static_utm_publisher src/static_utm_publisher.cpp src/utm_tf_node.cpp)
target_link_libraries(static_utm_publisher ${catkin_LIBRARIES})
add_dependencies(static_utm_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(dynamic_odom_estimator src/dynamic_odom_estimator_node.cpp src/dynamic_odom_estimator.cpp)
target_link_libraries(dynamic_odom_estimator ${catkin_LIBRARIES})
add_dependencies(dynamic_odom_estimator ${PROJECT_NAME}_generate_messages_cpp)
