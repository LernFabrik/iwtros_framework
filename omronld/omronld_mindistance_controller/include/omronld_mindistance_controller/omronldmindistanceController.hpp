#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>

namespace omronld_mindistance_controller {

/*!
 * Class containing the omronld minimal distance Controller
 */
class omronldmindistanceController {

  private:
    bool readParameters();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::NodeHandle nodeHandle_;
    ros::Subscriber scanSubscriber_;
    std::string scanTopic_;
    int subscriberQueueSize_;

  public:
    /*!
     * Constructor.
     */
    omronldmindistanceController(ros::NodeHandle& nodeHandle);

    /*!
     * Destructor.
     */
    virtual ~omronldmindistanceController();

};

}
