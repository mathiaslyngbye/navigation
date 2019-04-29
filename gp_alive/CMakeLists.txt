cmake_minimum_required(VERSION 2.8.3)
project(gp_alive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Num.msg
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gp_alive src/gp_alive.cpp)
target_link_libraries(gp_alive ${catkin_LIBRARIES})
