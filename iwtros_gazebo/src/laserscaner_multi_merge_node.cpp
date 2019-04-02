#include <iwtros_gazebo/multi_laserscaner.hpp>
#include <ros/ros.h>

int main (int argc, char** argv){
    ros::init(argc, argv, "laserscaner_multi_merge_node");
    ROS_INFO("Initialization is Start");
    iwtros::laserScanMerger merge("/deactivateScan");
    ros::spin();
    return 0;
}