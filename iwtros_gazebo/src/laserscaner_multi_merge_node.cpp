#include <iwtros_gazebo/multi_laserscaner.hpp>
#include <ros/ros.h>

int main (int argc, char** argv){
    ros::init(argc, argv, "laserscaner_multi_merge_node");
    ROS_INFO("Initialization is Start");
    iwtros::laserScanMerger merge(-3.1400001049,
                                    3.1400001049,
                                    0.00579999992624,
                                    0.0,
                                    0.0333,
                                    0.1,
                                    15.0,
                                    "/deactivateScan");
    ros::spin();
    return 0;
}