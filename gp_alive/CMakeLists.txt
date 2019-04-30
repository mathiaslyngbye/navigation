cmake_minimum_required(VERSION 2.8.3)
project(gp_alive)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED roscpp)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gp_manager
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(alive src/alive.cpp)
target_link_libraries(alive ${catkin_LIBRARIES})
