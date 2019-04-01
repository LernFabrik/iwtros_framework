#ifndef _FTS_CONTROLLER_HPP_
#define _FTS_CONTROLLER_HPP_

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <iwtros_msgs/ftsControl.h>

namespace iwtros{
    enum select_table{
        IIWA = 0,
        UR5 = 1,
        PANDA = 2
    };
    class ftsControl{
        ros::NodeHandle node_;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
        ros::Publisher tableControl_iiwaPub, tableControl_ur5Pub, tableControl_pandaPub, fts_goalPub;
        ros::Subscriber ftsOdom;
        tf2_ros::Buffer tf2Buffer;
        public:
            ftsControl();
            ~ftsControl();
            void goToTableLocation(select_table sel);
            void reverseDocking();
            void stopRobotTask();
            void ftsOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
            void ftsStartCallback(const iwtros_msgs::ftsControl::ConstPtr& msg);
        private:
            void setDynamicParam();
            void withTableDynamicParam();
            void resetDynamicParam();
            static void quadToEuler(geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw);
            void getTransforms(std::string parent, std::string child, geometry_msgs::TransformStamped& stamped);

            std::string iiwa_table_topic, ur5_table_topic, panda_table_topic;
            std::string worldFrame;
            std::string iiwaFrame, ur5Frame, pandaFrame;
            select_table seletTable;
            double roll, pitch, yaw;

            geometry_msgs::TransformStamped stampedtf2Cell;
            move_base_msgs::MoveBaseGoal goal, endGoal;
            geometry_msgs::PoseWithCovariance ftsPose;
            uint8_t chooseTable;

            bool lockCell;
    };
}

#endif
