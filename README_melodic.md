# iwtros_framework

**Current status**: Compilation done, not tested  

To shift from kinetic to melodic. Compilation fails due to the following reasons:  
1. New Gazebo API, refactor and rename  
  they refactor some stuff, like ignition name spaces  
2. move_group rename  
3. urdf related library use STL for shared_ptr instead of Boost library  


**ToDo**:  
1. Gazebo plugin in iwtros_gazeo, is currently ugly fixed, due to uuid or tinyxml not found by cmake  
[Discussion](https://bitbucket.org/ignitionrobotics/ign-cmake/issues/40/target-uuid-uuid-does-not-exist)  
[ignition cmake](https://bitbucket.org/ignitionrobotics/ign-cmake)  
