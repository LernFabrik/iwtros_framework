# iwtros_framework

**Current status**: Compilation done, not tested  

To shift from kinetic to melodic. Compilation fails due to the following reasons:  
1. New Gazebo API, refactor and rename  
  they refactor some stuff, like ignition name spaces  
2. [move_group rename](https://github.com/ros-planning/moveit/issues/37)  
3. urdf related library use STL for shared_ptr instead of Boost library  


**ToDo**:  
1. Gazebo plugin in iwtros_gazeo, is currently ugly fixed, due to uuid or tinyxml not found by cmake  
[Discussion](https://bitbucket.org/ignitionrobotics/ign-cmake/issues/40/target-uuid-uuid-does-not-exist)  
[Other possible solution](https://blog.csdn.net/zjq2008wd/article/details/17450033) add softlink  
[ignition cmake](https://bitbucket.org/ignitionrobotics/ign-cmake)  

libfraka build path setup:
catkin build -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/($ Path)/libfranka/build

