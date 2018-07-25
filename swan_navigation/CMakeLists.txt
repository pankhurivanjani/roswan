cmake_minimum_required(VERSION 2.8.3)
project(swan_navigation)

find_package(catkin REQUIRED COMPONENTS
    move_base
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
