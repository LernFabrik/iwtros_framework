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
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  iwtros_msgs
)

#catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES iwtros_goal 
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs move_base_msgs roscpp rospy sensor_msgs tf tf2  
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ur5_async_task_node 
  src/ur5_async_task.cpp
)
target_link_libraries(ur5_async_task_node 
  ${catkin_LIBRARIES}
)
############## IIWA 7 ##########################
add_executable(iiwa_sb_node 
  src/iiwa_sb.cpp
)
target_link_libraries(iiwa_sb_node 
  ${catkin_LIBRARIES}
)

add_executable(iiwa_task_handler_node
  src/iiwa_task_handler.cpp
)
target_link_libraries(iiwa_task_handler_node 
  ${catkin_LIBRARIES}
)

############### FTS Controller ################
# create PID library
add_library(reversePID
  src/reversePID.cpp
)
target_link_libraries(reversePID
  ${catkin_LIBRARIES}
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
  reversePID 
)

add_library(fts_controller
  src/fts_controller.cpp
)
target_link_libraries(fts_controller
  ${catkin_LIBRARIES}
)

add_executable(fts_controller_node 
  src/fts_controller_node.cpp
)
target_link_libraries(fts_controller_node 
  ${catkin_LIBRARIES}
  fts_controller
)



################ Panda Pick Place ###############
add_executable(panda_pick_place_node 
  src/panda_pick_place.cpp
)
target_link_libraries(panda_pick_place_node 
  ${catkin_LIBRARIES}
)