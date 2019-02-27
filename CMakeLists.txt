cmake_minimum_required(VERSION 2.4.6)
project(alfons_robot_bringup)
find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot3_msgs
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
