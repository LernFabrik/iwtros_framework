#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

<<<<<<< Updated upstream
enum robotState{
    STATE1 = 0,
    STATE2 = 1,
    STATE3 = 2
};

void getPoses(geometry_msgs::PoseStamped &poses, robotState state){
    poses.header.frame_id = "iiwa_link_0";
    poses.header.stamp = ros::Time::now() + ros::Duration(2.1);
    poses.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 3.14, 0);

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


=======
#include <fstream>


void storeNewPoses(){
    
}

>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

    geometry_msgs::PoseStamped target_pose;
    robotState state = STATE1;
    int counter = 0;

    while (!ros::isShuttingDown()){
        getPoses(target_pose, state);

        iiwa_group.setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit::planning_interface::MoveItErrorCode success = iiwa_group.plan(my_plan);
        ROS_INFO("plan: %s", success?"SUCCESS":"FAIL");
        if(success){
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
=======
}
>>>>>>> Stashed changes
