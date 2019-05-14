#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

enum robotState{
    STATE1 = 0,
    STATE2 = 1,
    STATE3 = 2
};

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface){
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "boundary1";
    collision_objects[0].header.frame_id = "iiwa_link_0";

    /* Collision object 1 */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.01;    
    collision_objects[0].primitives[0].dimensions[1] = 0.8;
    collision_objects[0].primitives[0].dimensions[2] = 0.5;    
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.35;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.25;
    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}


void getPoses(geometry_msgs::PoseStamped &poses, robotState state){
    poses.header.frame_id = "iiwa_link_0";
    poses.header.stamp = ros::Time::now() + ros::Duration(2.1);
    tf2::Quaternion q;
    q.setEuler(0, 0, M_PI/2);
    poses.pose.orientation.x = q.x();
    poses.pose.orientation.y = q.y();
    poses.pose.orientation.z = q.z();
    poses.pose.orientation.w = q.w();
    //poses.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0.0, M_PI/2);

    switch (state)
    {
        case STATE1:
            poses.pose.position.x = 0.0;
            poses.pose.position.y = 0.4;
            poses.pose.position.z = 0.4;
            break;
        
        case STATE2:
            poses.pose.position.x = -0.4;
            poses.pose.position.y = 0.0;
            poses.pose.position.z = 0.25;
            break;
        
        case STATE3:
            poses.pose.position.x = 0.4;
            poses.pose.position.y = 0.0;
            poses.pose.position.z = 0.25;
            break;
    
        default:
            break;
    };
}


int main(int argc, char** argv){
    ros::init(argc, argv, "iiwa_task_handler");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planing_scene_interface;
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

    ROS_INFO("End effector cartesion pose x: %lf , y: %lf, z: %lf", current_pose.pose.position.x,
                                                                current_pose.pose.position.y,
                                                                current_pose.pose.position.z);
    ROS_INFO("End effector cartesion Orientation x: %lf , y: %lf, z: %lf, w: %lf", current_pose.pose.orientation.x,
                                                                current_pose.pose.orientation.y,
                                                                current_pose.pose.orientation.z,
                                                                current_pose.pose.orientation.w);
    
    //addCollisionObject(planing_scene_interface);
    geometry_msgs::PoseStamped target_pose;
    robotState state = STATE1;
    int counter = 0;

    while (!ros::isShuttingDown()){
        //getPoses(target_pose, state);
        /*Testing with random pose generator*/
        target_pose = iiwa_group.getRandomPose(iiwa_group.getEndEffectorLink().c_str());
        /* TODO: Add Planning scense to restrict the motion of the robot
            if the URDF model spawned in the Gazebo and which is not in the robots moveit config
            then the system fails if the robot collide with the other models. */
        ROS_INFO("Target cartesion pose x: %lf , y: %lf, z: %lf", target_pose.pose.position.x,
                                                                target_pose.pose.position.y,
                                                                target_pose.pose.position.z);
        ROS_INFO("Target cartesion Orientation x: %lf , y: %lf, z: %lf, w: %lf", target_pose.pose.orientation.x,
                                                                target_pose.pose.orientation.y,
                                                                target_pose.pose.orientation.z,
                                                                target_pose.pose.orientation.w);

        iiwa_group.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveItErrorCode success = iiwa_group.plan(my_plan);
        ROS_INFO("plan: %s", success?"SUCCESS":"FAIL");
        if(success){
            ROS_INFO("Moving to the target goal");
            iiwa_group.move();
        }
        sleep(5);

        counter ++;
        switch (counter)
        {
            case 1:
                state = STATE2;
                break;
            
            case 2:
                state = STATE1;
                break;
            case 3:
                state = STATE3;
                break;
        
            default:
                counter = 0;
                break;
        }
        if(counter == 3) counter = 0;
    }
    
    
}
