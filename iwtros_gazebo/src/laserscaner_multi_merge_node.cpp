#include <iwtros_gazebo/multi_laserscaner.hpp>
#include <ros/ros.h>

int main (int argc, char** argv){
    ros::init(argc, argv, "laserscaner_multi_merge_node");

    iwtros::laserScanMerger merge(-3.1400001049,                //angle_min
                                    3.1400001049,               //angle_max
                                    0.00579999992624,           //angle_increment
                                    0.0,                        //time_increment
                                    0.0333,                     //scan_time
                                    0.1,                        //range_min
                                    15.0,                       //range_max
                                    "/deactivateScan");
    ros::spin();
    return 0;
}