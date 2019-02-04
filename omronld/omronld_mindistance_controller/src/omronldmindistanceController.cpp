#include "omronld_mindistance_controller/omronldmindistanceController.hpp"

namespace omronld_mindistance_controller {

omronldmindistanceController::omronldmindistanceController(ros::NodeHandle& nodeHandle) :
    nodeHandle_(),
    subscriberQueueSize_(),
    scanTopic_()

         {

              if (!readParameters()) {
                      ROS_ERROR("Could not read parameters.");
                      ros::requestShutdown();
                      }

              scanSubscriber_ = nodeHandle_.subscribe(scanTopic_, subscriberQueueSize_,
                                  &omronldmindistanceController::scanCallback,
                                  this);

         }

omronldmindistanceController::~omronldmindistanceController()
    {
    }

bool omronldmindistanceController::readParameters() {
    if (!nodeHandle_.getParam("/omronld_mindistance_controller/scan_subscriber_topic_name",
                              scanTopic_))
    return false;

    if (!nodeHandle_.getParam("/omronld_mindistance_controller/scan_subscriber_queue_size",
                    subscriberQueueSize_))
    return false;

    return true;

    }

void omronldmindistanceController::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {

        double min = msg->range_max;

        for (int i = 0; i < msg->ranges.size(); ++i) {
              if (msg->ranges[i] < min)
                min = msg->ranges[i];
            }

        ROS_INFO_STREAM("Minimum Range: " << min );

    }

}
