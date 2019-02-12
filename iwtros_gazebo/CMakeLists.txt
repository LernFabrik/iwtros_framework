cmake_minimum_required(VERSION 2.8.3)
project(iwtros_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  sensor_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
)

find_package(gazebo REQUIRED)

find_package(TinyXML REQUIRED)





catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    conveyor_spawner
    urdf_creator
  CATKIN_DEPENDS 
    cmake_modules 
    gazebo_msgs 
    gazebo_plugins
    gazebo_ros 
    geometry_msgs
    sensor_msgs 
    message_runtime 
    roscpp 
    rospy 
    std_msgs 
    std_srvs 
    tf
    DEPENDS
      TinyXML
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
  ${roscpp_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Create URDF creator libraries
add_library(urdf_creator 
  src/urdf_creator.cpp
)
target_link_libraries(urdf_creator 
  ${catkin_LIBRARIES} 
  ${TinyXML_LIBRARIES}
)

# Create conveyor spawner library
add_library(conveyor_spawner 
  src/conveyor_spawner.cpp
)
target_link_libraries(conveyor_spawner 
  ${catkin_LIBRARIES}  
  urdf_creator
)

# Create URDF creator  test node
add_executable(urdf_creator_test_node 
  src/urdf_creator_test.cpp
)
target_link_libraries(urdf_creator_test_node 
  ${catkin_LIBRARIES} 
  urdf_creator
)

# Create conveyor spawner executable
add_executable(conveyor_spawn_node  
  src/conveyor_spawn_node.cpp
)
target_link_libraries(conveyor_spawn_node  
  ${catkin_LIBRARIES}  
  conveyor_spawner
)

######################## Make plugin libraries #######################################
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
## libROSTableControlPlugin.so
set(ros_table_control_plugins_name ROSTableControlPlugin)
add_library(${ros_table_control_plugins_name} SHARED src/plugins/ROSTableControlPlugin.cc)
target_link_libraries(${ros_table_control_plugins_name} ${GAZEBO_LIBRARIES})

#####################################################################################