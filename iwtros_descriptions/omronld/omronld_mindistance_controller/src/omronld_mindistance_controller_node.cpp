#include <ros/ros.h>
#include "omronld_mindistance_controller/omronldmindistanceController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "omronld_mindistance_controller");

  ros::NodeHandle nodeHandle;

  omronld_mindistance_controller::omronldmindistanceController omronldmindistanceController(nodeHandle);

  ros::spin();
  return 0;
}
