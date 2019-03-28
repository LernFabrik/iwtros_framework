#include <iwtros_gazebo/multi_laserscaner.hpp>

namespace iwtros{
    laserScanMerger::laserScanMerger(double angle_min,
                            double angle_max,
                            double angle_increment,
                            double time_increment,
                            double scan_time,
                            double range_min,
                            double range_max,
                            std::string deactivatingTopic){
        ros::NodeHandle nh("~");
        
        //nh.getParam("destination_frame", destination_frame);
        //nh.getParam("cloud_destination_topic", cloud_destination_topic);
        //nh.getParam("scan_destination_topic", scan_destination_topic);
        //nh.getParam("laserscan_topics_front", laserscan_topic_front);
        //nh.getParam("laserscan_topics_back", laserscan_topic_back);
        destination_frame = "base_link";
        cloud_destination_topic = "/merged_cloud";
        scan_destination_topic = "/scan";
        laserscan_topic_front = "/scanFront";
        laserscan_topic_back = "/scanBack";
        
        front_scan_subscriber_ = node_.subscribe<sensor_msgs::LaserScan> (laserscan_topic_front.c_str(), 1, boost::bind(&laserScanMerger::scanCallback, this, _1, laserscan_topic_front.c_str()));
        back_scan_subscriber_ = node_.subscribe<sensor_msgs::LaserScan> (laserscan_topic_back.c_str(), 1, boost::bind(&laserScanMerger::scanCallback, this, _1, laserscan_topic_back.c_str()));
        ROS_INFO("Initialization is start2");
        clouds_modified.push_back(false);
        clouds_modified.push_back(false);
        clouds.resize(2);
        deactivation_subscriber_ = node_.subscribe<std_msgs::Bool> (deactivatingTopic.c_str(), 1, boost::bind(&laserScanMerger::deactivateBackScanCallback, this, _1));
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (cloud_destination_topic.c_str(), 1, false);
        laser_scaner_publisher_ = node_.advertise<sensor_msgs::LaserScan> (scan_destination_topic.c_str(), 1, false);
        tfListener_.setExtrapolationLimit(ros::Duration(0.1));

        this->angle_min = angle_min;
        this->angle_max = angle_max;
        this->angle_increment = angle_increment;
        this->time_increment = time_increment;
        this->scan_time = scan_time;
        this->range_min = range_min;
        this->range_max = range_max;
        ROS_INFO("Initialization is complete");
    }

    void laserScanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, const std::string &topic){
        sensor_msgs::PointCloud tmpCloud1, tmpCloud2;
        sensor_msgs::PointCloud2 tmpCloud3;

        // Verify that TF knows how to transform from the received scan to the destination scan frame
        tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));
        
        projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_);
        try
        {
            tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
        }
        catch(tf::TransformException e)
        {
            ROS_ERROR("%s", e.what());
            return;
        }
        ROS_INFO("Scan callback topic: %s", topic.c_str());
        if(topic == "/scanFront"){
            ROS_INFO("Scan callback 1 - 1");
            sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2, tmpCloud3);
            pcl_conversions::toPCL(tmpCloud3, clouds[0]);
            ROS_INFO("Scan callback 1");
            clouds_modified[0] = true;
        }
        if(topic == "/scanBack"){
            ROS_INFO("Scan callback 2 -2");
            sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2, tmpCloud3);
            pcl_conversions::toPCL(tmpCloud3, clouds[1]);
            ROS_INFO("Scan callback 2");
            clouds_modified[1] = true;
        }
        
        ROS_INFO("Scan callback Stage 1");
        //Cout number of scans
        int totalClouds = 0;
        for(int i=0; i<clouds_modified.size(); ++i){
            if(clouds_modified[i]){
                ++totalClouds;
            }
        }

        //Go ahead only if all subscribed scans have arrived
        if(totalClouds == clouds_modified.size()){
            pcl::PCLPointCloud2 merged_cloud = clouds[0];
            clouds_modified[0] = false;

            for(int i=1; i<clouds_modified.size();i++){
                pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
                clouds_modified[i] = false;
            }

            point_cloud_publisher_.publish(merged_cloud);

            Eigen::MatrixXf points;
            pcl::getPointCloudAsEigen(merged_cloud, points);

            laserScanMerger::pointcloud_to_laserscan(points, &merged_cloud);
        }
    }

    void laserScanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud){
        sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
        output->header =pcl_conversions::fromPCL(merged_cloud->header);
        output->header.frame_id = destination_frame.c_str();
        output->header.stamp = ros::Time::now();
        output->angle_min = this->angle_min;
        output->angle_max = this->angle_max;
        output->angle_increment = this->angle_increment;
        output->time_increment = this->time_increment;
        output->scan_time = this->scan_time;
        output->range_min = this->range_min;
        output->range_max = this->range_max;

        uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
        output->ranges.assign(ranges_size, output->angle_max + 1.0);

        for(int i = 0; i<points.cols(); i++){
            const float &x = points(0, i);
            const float &y = points(1, i);
            const float &z = points(2, i);

            if(std::isnan(x) || std::isnan(y) || std::isnan(z)){
                ROS_DEBUG("rejected for nan in points(%f, %f, %f)\n", x,y,z);
                continue;
            }

            double range_sq = y*y+x*x;
            double range_min_sq_ = output->range_min * output->range_min;
            if (range_sq < range_min_sq_) {
                ROS_DEBUG("reject for range %f below minimum value %f. Point:(%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
                continue;
            }

            double angle = atan2(y, x);
            if (angle < output->angle_min || angle > output->angle_max) {
                ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			    continue;
            }
            int index = (angle - output->angle_min) / output->angle_increment;
            if(output->ranges[index] * output->ranges[index] > range_sq){
                output->ranges[index] = sqrt(range_sq);
            }
        }
        laser_scaner_publisher_.publish(output);
    }

    void laserScanMerger::deactivateBackScanCallback(const std_msgs::Bool::ConstPtr& detach){
        bool deactivate = detach->data;
        if(deactivate == true){
            back_scan_subscriber_.shutdown();
            clouds_modified.resize(1);
            this->tmp_angle_min = this->angle_min;
            this->tmp_angle_max = this->angle_max;
            this->angle_min = -2.09439992905;
            this->angle_max = 2.09439992905;
        }
        else if (deactivate == false){
            back_scan_subscriber_ = node_.subscribe<sensor_msgs::LaserScan> (laserscan_topic_back.c_str(), 1, boost::bind(&laserScanMerger::scanCallback, this, _1, laserscan_topic_back));
            clouds_modified.resize(2);
            this->angle_min = this->tmp_angle_min;
            this->angle_max = this->tmp_angle_max;
        }
        
    }
}