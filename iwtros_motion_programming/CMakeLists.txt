cmake_minimum_required(VERSION 2.8.3)
project(iwtros_motion_programming)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  kdl_conversions
  moveit_core
  moveit_msgs
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  pilz_extensions
  pilz_msgs
  pluginlib
  roscpp
  rospy
  tf
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_geometry_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iwtros_motion_programming
#  CATKIN_DEPENDS eigen_conversions geometry_msgs kdl_conversions moveit_core moveit_msgs moveit_ros_move_group moveit_ros_planning moveit_ros_planning_interface pilz_extensions pilz_msgs pluginlib roscpp rospy tf tf2 tf2_eigen tf2_geometry_msgs tf2_geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

