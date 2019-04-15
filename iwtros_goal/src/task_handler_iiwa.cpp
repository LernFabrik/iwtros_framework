#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "iiwa_task_handler");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface iiwa_group("iiwa_arm");
    iiwa_group.setMaxVelocityScalingFactor(1.0);
    iiwa_group.setMaxAccelerationScalingFactor(1.0);
    iiwa_group.setPlannerId("RRTConnectkConfigDefault");
    iiwa_group.setPoseReferenceFrame("iiwa_link_0");
    iiwa_group.setStartStateToCurrentState();
    ROS_ERROR("Planning refence frame:%s", iiwa_group.getPlanningFrame().c_str());
    ROS_ERROR("End effector reference frame: %s", iiwa_group.getEndEffectorLink().c_str());

    geometry_msgs::PoseStamped current_pose;

    current_pose = iiwa_group.getCurrentPose(iiwa_group.getEndEffectorLink().c_str());

    ROS_INFO("End effector cartesion pose x: %d , y: %d, z: %d", current_pose.pose.position.x,
                                                                current_pose.pose.position.y,
                                                                current_pose.pose.position.z);
    ROS_INFO("End effector cartesion Orientation x: %d , y: %d, z: %d, w: %d", current_pose.pose.orientation.x,
                                                                current_pose.pose.orientation.y,
                                                                current_pose.pose.orientation.z,
                                                                current_pose.pose.orientation.w);
}