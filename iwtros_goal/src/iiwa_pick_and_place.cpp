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
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


static std::string PLANNING_GROUP = "iiwa_arm";

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

/** Using planning scene interface the collision objects are attached
 * @breif the size and poses are need to be redifined
 */
void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
    std::vector<moveit_msgs::CollisionObject> collision_obj;
    collision_obj.resize(1);
    collision_obj[0].id = "object";
    collision_obj[0].header.frame_id = "iiwa_link_0";
    collision_obj[0].primitives.resize(1);
    collision_obj[0].primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
    collision_obj[0].primitives[0].dimensions.resize(2);
    collision_obj[0].primitives[0].dimensions[0] = 0.02;
    collision_obj[0].primitives[0].dimensions[1] = 0.02;
    collision_obj[0].primitive_poses.resize(1);
    collision_obj[0].primitive_poses[0].position.x = 0.0;
    collision_obj[0].primitive_poses[0].position.y = -0.5;
    collision_obj[0].primitive_poses[0].position.z = 0.01;
    collision_obj[0].operation = collision_obj[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_obj);

}

void ExecutePose(moveit::planning_interface::MoveGroupInterface& iiwa_group, geometry_msgs::PoseStamped target_pose){
    const robot_state::JointModelGroup* joint_model_group = 
                                iiwa_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    /*Visual tool initialization */
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link_ee");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 0.2;
    visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    moveit_msgs::OrientationConstraint oriCon;
    oriCon.header.frame_id = "iiwa_link_0";
    oriCon.link_name = "iiwa_link_ee";
    oriCon.orientation = target_pose.pose.orientation;
    oriCon.absolute_x_axis_tolerance = 0.01;
    oriCon.absolute_y_axis_tolerance = 0.01;
    oriCon.absolute_z_axis_tolerance = 0.01;
    oriCon.weight = 1.0;

    moveit_msgs::Constraints planConstraints;
    planConstraints.orientation_constraints.push_back(oriCon);
    //iiwa_group.setPathConstraints(planConstraints);
    robotSetupChecker();
    
    robot_state::RobotState start_State(*iiwa_group.getCurrentState());
    geometry_msgs::PoseStamped getPose = iiwa_group.getCurrentPose("iiwa_link_ee");
    geometry_msgs::Pose startPose;
    ROS_INFO("Robot ee pose: %f", getPose.pose.position.z);
    startPose.orientation = getPose.pose.orientation;
    startPose.position = getPose.pose.position;
    start_State.setFromIK(joint_model_group, startPose);
    iiwa_group.setStartState(start_State);                    // Optional setting: If set not woring fines

    moveit_msgs::TrajectoryConstraints motionReq;
    motionReq.constraints.resize(1);
    motionReq.constraints[0].orientation_constraints.push_back(oriCon);
    motionReq.constraints[0].position_constraints.resize(1);
    motionReq.constraints[0].position_constraints[0].header.frame_id = "iiwa_link_0";
    motionReq.constraints[0].position_constraints[0].link_name = "iiwa_link_ee";
    motionReq.constraints[0].position_constraints[0].constraint_region.primitive_poses.resize(1);
    motionReq.constraints[0].position_constraints[0].constraint_region.primitive_poses[0] = target_pose.pose;
    motionReq.constraints[0].position_constraints[0].constraint_region.primitives.resize(1);
    motionReq.constraints[0].position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
    motionReq.constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions.push_back(2e-3);

    iiwa_group.setTrajectoryConstraints(motionReq);

    iiwa_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode eCode = iiwa_group.plan(my_plan);
    ROS_INFO("Motion planning is: %s", eCode?"Success":"Failed");

    /*Visualize the plan before the execution */
    visual_tools.publishAxisLabeled(target_pose.pose, "Pose");
    visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    if(eCode){
        iiwa_group.execute(my_plan);
    }

    iiwa_group.clearPathConstraints();
    iiwa_group.clearPoseTarget();
    iiwa_group.clearPoseTargets();
    iiwa_group.clearTrajectoryConstraints();
}


int main(int argc, char** argv){
    ros::init(argc, argv, "iiwa_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner sppinner(1);
    sppinner.start();
    
    // robotSetupChecker();
    
    moveit::planning_interface::MoveGroupInterface iiwa_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    iiwa_group.setPlannerId("PTP");
    iiwa_group.setMaxVelocityScalingFactor(0.2);
    iiwa_group.setMaxAccelerationScalingFactor(0.2);
    iiwa_group.setPoseReferenceFrame("iiwa_link_0");
    iiwa_group.setEndEffectorLink("iiwa_link_ee");
    iiwa_group.setNumPlanningAttempts(10);
    
    ROS_WARN("IIWA reference frame: %s", iiwa_group.getPlanningFrame().c_str());
    //ROS_WARN("IIWA end effector: %s", iiwa_group.getEndEffector().c_str());
    ROS_WARN("IIWA End effecotor link: %s", iiwa_group.getEndEffectorLink().c_str());

    //iiwa_group.setNamedTarget("iiwa_Pose1");
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "iiwa_link_0";
    target_pose.header.stamp = ros::Time::now()+ros::Duration(2.1);
    target_pose.pose.position.x = 0.4;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = 1.4;
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, M_PI/2);

    geometry_msgs::PoseStamped target_pose2;
    target_pose2.header.frame_id = "iiwa_link_0";
    target_pose2.header.stamp = ros::Time::now() + ros::Duration(2.1);
    target_pose2.pose.position.x = -0.4;
    target_pose2.pose.position.y = 0.2;
    target_pose2.pose.position.z = 1.4;
    target_pose2.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);

    addCollisionObject(planning_scene_interface);

    /* Refere the Readme*/
    while (ros::ok())
    {
        ExecutePose(iiwa_group, target_pose);
        ros::Duration(0.5).sleep();
        ExecutePose(iiwa_group, target_pose2);
        ros::Duration(0.5).sleep();
    }
    ros::waitForShutdown();
    return 0;
}