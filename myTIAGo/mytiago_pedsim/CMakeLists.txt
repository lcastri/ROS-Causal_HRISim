cmake_minimum_required(VERSION 2.8.3)
project(mytiago_pedsim)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

# Set CMAKE_PREFIX_PATH
# set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /root/ros_ws/src/pedsim_ros/pedsim_msgs)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  gazebo_msgs
  pedsim_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
 find_package(gazebo REQUIRED )
 find_package(pedsim_msgs REQUIRED)
 link_directories(${GAZEBO_LIBRARY_DIRS})


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
  CATKIN_DEPENDS gazebo_ros roscpp  geometry_msgs pedsim_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


add_library(myActorPosesPlugin src/actor_poses_plugin.cpp)
target_link_libraries(myActorPosesPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(myActorPosesPlugin ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(
  TARGETS myActorPosesPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

