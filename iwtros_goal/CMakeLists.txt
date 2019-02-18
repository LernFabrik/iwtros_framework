cmake_minimum_required(VERSION 2.8.3)
project(iwtros_goal)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  tf2
)

#catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iwtros_goal
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs move_base_msgs roscpp rospy sensor_msgs tf tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fts_goal_node 
  src/fts_goal_node.cpp
)
target_link_libraries(fts_goal_node 
  ${catkin_LIBRARIES} 
)

add_executable(control_table_node 
  src/control_table_node.cpp
)
target_link_libraries(control_table_node 
  ${catkin_LIBRARIES} 
)
