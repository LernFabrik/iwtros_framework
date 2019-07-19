#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/Constraints.h>
#include <shape_msgs/SolidPrimitive.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* Robot model and robot states*/
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

void robotSetupChecker(){
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("iiwa_arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("iiwa_link_ee");

    std::size_t attempts = 10;
    double timeout = 0.1;
    std::vector<double> joint_values;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, attempts, timeout);
    if(found_ik){
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i=0; i<joint_names.size();i++){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }else{
        ROS_ERROR("No IK solution is found");
    }
}

template <typename T_pose, typename T_ref, typename T_targ>
void _to_orin_contr(moveit_msgs::OrientationConstraint& oriCon, T_pose goal_pose, T_ref referenceFrame, T_targ targetFrame){
    //moveit_msgs::OrientationConstraint oriCon;
    oriCon.header.frame_id = referenceFrame;
    oriCon.link_name = targetFrame;
    oriCon.orientation = goal_pose.orientation;
    oriCon.absolute_x_axis_tolerance = 1e-5;
    oriCon.absolute_y_axis_tolerance = 1e-5;
    oriCon.absolute_z_axis_tolerance = 1e-5;
    oriCon.weight = 1;
    //return oriCon;
}

template <typename T_pose, typename T_ref, typename T_targ>
void _to_pose_contr(moveit_msgs::PositionConstraint& poseCon,  T_pose goal_pose, T_ref referenceFrame, T_targ targetFrame){
    //moveit_msgs::PositionConstraint poseCon;
    poseCon.header.frame_id = referenceFrame;
    poseCon.link_name = targetFrame;
    poseCon.constraint_region.primitive_poses.push_back(goal_pose);
    poseCon.weight = 1;
    
    shape_msgs::SolidPrimitive region;
    region.type = shape_msgs::SolidPrimitive::SPHERE;
    region.dimensions.push_back(2e-3);
    poseCon.constraint_region.primitives.push_back(region);
    //return poseCon;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "iiwa_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner sppinner(1);
    sppinner.start();
    
    robotSetupChecker();

    moveit::planning_interface::MoveGroupInterface iiwa_group("iiwa_arm");
    iiwa_group.setPlannerId("PTP");
    iiwa_group.setMaxVelocityScalingFactor(0.2);
    iiwa_group.setMaxAccelerationScalingFactor(0.2);
    iiwa_group.setPoseReferenceFrame("iiwa_link_0");
    iiwa_group.setEndEffectorLink("iiwa_link_ee");
    
    ROS_WARN("IIWA reference frame: %s", iiwa_group.getPlanningFrame().c_str());
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

    /**
     * Predifined pose is working however the problem occurs during the cartesian pose
     * @solution: Currently problem is narrow down to motion planning request
     * detailed decoding the code :) found in ($PROJECT)/pilz_industrial_motion/pilz_robot_programming/src/commands.py (200)
     */

    moveit_msgs::MotionPlanRequest planning_req;
    planning_req.planner_id = "PTP";
    planning_req.group_name = "iiwa_arm";
    planning_req.max_velocity_scaling_factor = 0.2;
    planning_req.max_acceleration_scaling_factor = 0.2;
    planning_req.allowed_planning_time = 1.0;
    planning_req.start_state.is_diff = true;

    moveit_msgs::Constraints planning_contr;
    moveit_msgs::PositionConstraint poseContr;
    moveit_msgs::OrientationConstraint orienContr;
    _to_orin_contr(orienContr, target_pose.pose, iiwa_group.getPlanningFrame(), iiwa_group.getEndEffectorLink());
    _to_pose_contr(poseContr,target_pose.pose, iiwa_group.getPlanningFrame(), iiwa_group.getEndEffectorLink());
    //planning_contr.orientation_constraints.resize(1);
    planning_contr.orientation_constraints.push_back(orienContr);
    planning_contr.position_constraints.push_back(poseContr);
    planning_req.goal_constraints.push_back(planning_contr);

    iiwa_group.constructMotionPlanRequest(planning_req);

    //iiwa_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode eCode = iiwa_group.plan(my_plan);
    ROS_INFO("Motion planning is: %s", eCode?"Success":"Failed");
    if(eCode){
        iiwa_group.execute(my_plan);
    }
    ros::waitForShutdown();
    return 0;
}