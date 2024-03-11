cmake_minimum_required(VERSION 2.8.3)
project(rviz_visualisation)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

# Set CMAKE_PREFIX_PATH
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /root/ros_ws/src/pedsim_ros/pedsim_msgs)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  pedsim_msgs
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gzb_vel_plugin
  CATKIN_DEPENDS geometry_msgs visualization_msgs pedsim_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


#############
## Install ##
#############


