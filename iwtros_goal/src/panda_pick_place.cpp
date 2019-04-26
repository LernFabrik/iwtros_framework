#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(trajectory_msgs::JointTrajectory& posture){
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    // Set as open, wide enough if panda robot;
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closeGripper(trajectory_msgs::JointTrajectory& posture){
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    // Set as open, wide enough if panda robot;
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.0;
    posture.points[0].positions[1] = 0.0;
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
    grasps[0].grasp_pose.pose.position.z = 0.125;
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);

    // Setting pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    /*Direction is set to positive z axis*/
    grasps[0].pre_grasp_approach.direction.vector.z = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.1;
    grasps[0].pre_grasp_approach.desired_distance = 0.120;

    // Setting post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link8";
    /* Test this before deploying in Hardware*/
    grasps[0].post_grasp_retreat.direction.vector.z = - 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting up figure in open positon before grasp

    // Close the fingure after the grasp

    move_group.setSupportSurfaceName("PandaTablePick");
    move_group.pick("object", grasps);
}

void place(moveit::planning_interface::MoveGroupInterface& move_group){
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting up place location pose
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, -M_PI/4);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(q);
    place_location[0].place_pose.pose.position.x = 0.48;
    place_location[0].place_pose.pose.position.y = - 0.2;
    place_location[0].place_pose.pose.position.z = 0.125;


    //Setting up pre pllace approach
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    // Direction is set to positive z direction
    place_location[0].pre_place_approach.direction.vector.z = 1.0;
    place_location[0].pre_place_approach.min_distance = 0.1;
    place_location[0].pre_place_approach.desired_distance = 0.12;

    // Setting post grasp retreat
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    place_location[0].post_place_retreat.direction.vector.z = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    //Setting posture of the eef after placing the object

    move_group.setSupportSurfaceName("PandaTablePlace");
    move_group.place("object", place_location); 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "panda_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner sppinner(1);
    sppinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    move_group.setPlanningTime(45.0);

    //add collision object with planning scene
    ros::WallDuration(1.0).sleep();

    pick(move_group);

    ros::WallDuration(1.0).sleep();

    place(move_group);

    ros::waitForShutdown();
    return 0;
}