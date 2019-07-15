#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* Robot model and robot states*/
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

void openGripper(trajectory_msgs::JointTrajectory& posture){
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    // Set as open, wide enough if panda robot;
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    /*posture.points[0].effort.resize(2);
    posture.points[0].effort[0] = 0;
    posture.points[0].effort[1] = 0;*/
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closeGripper(trajectory_msgs::JointTrajectory& posture){
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    // Set them as closed;
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.0;
    posture.points[0].positions[1] = 0.0;
    /*posture.points[0].effort.resize(2);
    posture.points[0].effort[0] = 100;
    posture.points[0].effort[1] = 100;*/
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group){
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    //Setting up grasp pose
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0, -M_PI/4);
    grasps[0].grasp_pose.pose.position.x = 0.43;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.128;
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

    // Setting pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    /*Direction is set to positive z axis*/
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    /* Test this before deploying in Hardware*/
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.095;
    grasps[0].post_grasp_retreat.desired_distance = 0.15;

    // Setting up figure in open positon before grasp
    openGripper(grasps[0].pre_grasp_posture);

    // Close the fingure after the grasp
    closeGripper(grasps[0].grasp_posture);

    move_group.setSupportSurfaceName("panda_table_ee");
    move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group){
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting up place location pose
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(q);
    place_location[0].place_pose.pose.position.x = 0.3;
    place_location[0].place_pose.pose.position.y = -0.30;
    place_location[0].place_pose.pose.position.z = 0.05;


    //Setting up pre pllace approach
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    // Direction is set to positive z direction
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post grasp retreat
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    //Setting posture of the eef after placing the object
    openGripper(place_location[0].post_place_posture);

    move_group.setSupportSurfaceName("panda_table_ee");
    move_group.place("object", place_location); 
}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "object";
    collision_objects[0].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.01;    // height
    collision_objects[0].primitives[0].dimensions[1] = 0.0295;    // radius

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.43;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.03;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // Table
    /*collision_objects[1].id = "Table";
    collision_objects[1].header.frame_id = "panda_link0";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.75;    
    collision_objects[1].primitives[0].dimensions[1] = 0.75;  
    collision_objects[1].primitives[0].dimensions[2] = 0.5;  
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = -0.25;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;*/

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "panda_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner sppinner(1);
    sppinner.start();

    /* Robot model and robot states: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/robot_model_and_robot_state/robot_model_and_robot_state_tutorial.html*/
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model name: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i){
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    // Set joint limits
    joint_values[1] = 1.4;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
    // Formward Kinematics
    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");
    ROS_INFO("............Forward Kinematic............");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotational: \n" << end_effector_state.rotation() << "\n");
    // Inverse Kinematic
    std::size_t attempts = 10;
    double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, attempts, timeout);
    ROS_INFO("..............Inverse Kinematic..........");
    if(found_ik){
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i = 0; i < joint_names.size(); ++i){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }
    else{
        ROS_INFO("Did not find IK solution");
    }
    // Jacobian
    Eigen::Vector3d reference_point_positon(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group, 
                                kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                reference_point_positon, jacobian);
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
    /*******************************************************************/
    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit_msgs::MotionPlanRequest motion_planning;
    motion_planning.planner_id = "PTP";
    motion_planning.group_name = "panda_arm";
    motion_planning.max_velocity_scaling_factor = 0.2;
    motion_planning.max_acceleration_scaling_factor = 0.2;
    motion_planning.start_state.is_diff = true;
    //move_group.setPlanningTime(45.0);
    move_group.setPlannerId("PTP");
    move_group.setNamedTarget("panda_Pick");
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setStartStateToCurrentState();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode eCode = move_group.plan(my_plan);
    ROS_ERROR("Planning the motion is: %s", eCode?"succuss":"failed");
    if(eCode){
        std::stringstream strr;
        strr << eCode;
        ROS_ERROR("Unable to move: ");
    }
    //add collision object with planning scene
    addCollisionObject(planning_scene_interface);
    
    while (ros::ok()){
        ros::WallDuration(2.0).sleep();
        pick(move_group);
        ros::WallDuration(1.0).sleep();
        place(move_group);
    }

    ros::waitForShutdown();
    return 0;
}