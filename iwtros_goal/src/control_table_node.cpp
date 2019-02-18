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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "control_table_node");
    ros::NodeHandle nh;
    /* here require defination for tf are set*/
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    /*---------------------------------------*/

    ros::Rate rate(20.0);

    while(ros::ok()){
        geometry_msgs::TransformStamped stampedTfIIWA;
        geometry_msgs::TransformStamped stampedTfUR5;
        try{
            stampedTfIIWA = tfBuffer.lookupTransform("world", "iiwa_table_base", ros::Time(0));
        }catch(tf2::TransformException &e){
            ROS_WARN("%s", e.what());
            ros::Duration(0.1).sleep();
            continue;
        }
    }
    return 0;
}