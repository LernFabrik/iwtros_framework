#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "iiwa_mover");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface iiwa_group("iiwa_arm");
    iiwa_group.setMaxVelocityScalingFactor(0.5);

    iiwa_group.setPlannerId("RRTConnectkConfigDefault");                    //Specify planner to be used for the further planning
    iiwa_group.setPoseReferenceFrame("iiwa_link_0");                        //Specify which reference frame to assume for poses specified without a reference frame
    iiwa_group.setStartStateToCurrentState();                               //Ste the starting state for plannig to be that reported by the robot's joint state publicatioin
    ROS_INFO("Planning refence frame:%s", iiwa_group.getPlanningFrame().c_str());
    ROS_INFO("End effector reference frame: %s", iiwa_group.getEndEffectorLink().c_str());

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "iiwa_link_0";
    target_pose.header.stamp = ros::Time::now() + ros::Duration(2.1);
    target_pose.pose.position.x = -0.3;
    target_pose.pose.position.y = -0.4;
    target_pose.pose.position.z = 0.25;
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 3.14, 0.0);
    
    iiwa_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success =  iiwa_group.plan(my_plan);
    ROS_INFO("plan: %s", success?"SUCCESS":"FAIL");
    if(success){
        ROS_INFO("Moving......");
        iiwa_group.move();
    }
    sleep(5);

    while(!ros::isShuttingDown()){
        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose new_eef_pose;
        /*TO BE Continued*/
    }
    
    return 0;
}
