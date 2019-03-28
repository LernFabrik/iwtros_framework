#ifndef _MULTI_LASERSCANER_HPP_
#define __MULTI_LASERSCANER_HPP_

#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "pcl_ros/point_cloud.h"
#include <dynamic_reconfigure/server.h>

namespace iwtros{
    class laserScanMerger{
        public:
            laserScanMerger(double angle_min,
                            double angle_max,
                            double angle_increment,
                            double time_increment,
                            double scan_time,
                            double range_min,
                            double range_max,
                            std::string deactivatingTopic);
            void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, const std::string &topic);
            void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
            void deactivateBackScanCallback(const std_msgs::Bool::ConstPtr& detach);
        
        private:
            ros::NodeHandle node_;
            laser_geometry::LaserProjection projector_;
            tf::TransformListener tfListener_;

            ros::Publisher point_cloud_publisher_;
            ros::Publisher laser_scaner_publisher_;
            ros::Subscriber front_scan_subscriber_;
            ros::Subscriber back_scan_subscriber_;
            ros::Subscriber deactivation_subscriber_;
            std::vector<bool> clouds_modified;

            std::vector<pcl::PCLPointCloud2> clouds;
            std::vector<std::string> input_topic;

            void laser_scan_topic_paser();   // dont know if it requires

            double angle_min, tmp_angle_min;
            double angle_max, tmp_angle_max;
            double angle_increment;
            double time_increment;
            double scan_time;
            double range_min;
            double range_max;

            std::string destination_frame;
            std::string cloud_destination_topic;
            std::string scan_destination_topic;
            std::string laserscan_topic_front;
            std::string laserscan_topic_back;
    };
}


#endif //_MULTI_LASERSCANER_HPP_