#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* Robot model and robot states*/
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "iiwa_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner sppinner(1);
    sppinner.start();

    moveit::planning_interface::MoveGroupInterface iiwa_group("iiwa_arm");
    iiwa_group.setPlannerId("PTP");
    iiwa_group.setMaxVelocityScalingFactor(0.2);
    iiwa_group.setMaxAccelerationScalingFactor(0.2);
    iiwa_group.setPoseReferenceFrame("iiwa_link_0");
    iiwa_group.setEndEffectorLink("iiwa_link_ee");

    ROS_WARN("IIWA reference frame: %s", iiwa_group.getPoseReferenceFrame().c_str());
    //ROS_WARN("IIWA end effector: %s", iiwa_group.getEndEffector().c_str());
    ROS_WARN("IIWA End effecotor link: %s", iiwa_group.getEndEffectorLink().c_str());

    //iiwa_group.setNamedTarget("iiwa_Pose1");
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "iiwa_link_0";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = 0.4;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    iiwa_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode eCode = iiwa_group.plan(my_plan);
    ROS_INFO("Motion planning is: %s", eCode?"Success":"Failed");
    if(eCode){
        iiwa_group.execute(my_plan);
    }
    ros::waitForShutdown();
    return 0;
}