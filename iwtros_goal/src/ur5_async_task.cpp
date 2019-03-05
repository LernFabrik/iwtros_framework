#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <tf/tf.h>

int main(int argc, char** argv){
    moveit::planning_interface::MoveGroupInterface move_group("ur5_arm");
    move_group.setPoseReferenceFrame();
}