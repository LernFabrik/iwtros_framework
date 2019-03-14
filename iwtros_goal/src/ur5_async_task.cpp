#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <tf/tf.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "ur5_async_motion");
    moveit::planning_interface::MoveGroupInterface ur5_group("ur5_arm");
    /*Refere http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html#a92c9875f9f00a95693b3731d65a66ce1*/
    std::vector<std::string> namedTargets = ur5_group.getNamedTargets(); // Get the nameds of the named robot states available as targets, both either remmbered state or default from SRDF
    ur5_group.setNamedTarget("ur5_Pick"); // Set the current ... or if not found, that are specified in the SRDF under the name "name" as a group stete.
    ur5_group.asyncMove(); // Plan and execute a trajectory that takes the group of joint declared in the contructor to the specified traget. This call is always blocking(does not wait for the execution of the trjectory to complete)
}