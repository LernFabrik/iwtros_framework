cmake_minimum_required(VERSION 3.5.2)
project(iwtros_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  #cmake_modules
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
  laser_geometry
  pcl_ros
  dynamic_reconfigure
)

find_package(gazebo REQUIRED)
#find_package(TinyXML REQUIRED)
set(TinyXML_INCLUDE_DIRS /usr/include)
set(TinyXML_LIBRARIES /usr/lib/x86_64-linux-gnu/libtinyxml.so.2.6.2)

########### > before build run chmod a+x laserscan_multi_merger.cfg #############
generate_dynamic_reconfigure_options(cfg/laserscan_multi_merger.cfg)

catkin_package(
  INCLUDE_DIRS 
    include
    cfg
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
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
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


############ LASER SCAN MERGER ###############
add_library(multi_laserscaner 
  src/multi_laserscaner.cpp
)
target_link_libraries(multi_laserscaner 
  ${catkin_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
)

add_executable(laserscaner_multi_merge_node  
  src/laserscaner_multi_merge_node.cpp
)
target_link_libraries(laserscaner_multi_merge_node  
  ${catkin_LIBRARIES}  
  multi_laserscaner
)
add_dependencies(laserscaner_multi_merge_node ${PROJECT_NAME}_gencfg)

add_executable(laserscan_multi_merger_node  
  src/laserscan_multi_merger_node.cpp
)
target_link_libraries(laserscan_multi_merger_node  
  ${catkin_LIBRARIES}
)
add_dependencies(laserscan_multi_merger_node ${PROJECT_NAME}_gencfg)
##############################################