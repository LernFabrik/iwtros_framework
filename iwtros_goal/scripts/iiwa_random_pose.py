#!/usr/bin/env python
#This file created and modified for the purpose of 
# ROS-framework for LernFabrik work place simulation 
# Author: Vishnuprasad Prachandabhanu

import copy
import sys

import actionlib
import geometry_msgs
import moveit_commander
import moveit_msgs.msg
import rospy

def iiwa_pose_place():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('iiwa_random_motion', anonymous=True)

    iiwa_group = moveit_commander.MoveGroupCommander("iiwa_arm")

    iiwa_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    iiwa_client.wait_for_server()
    rospy.loginfo("Execution Trajectroy server is available for iiwa")

    rate = rospy.Rate(0.5)
    
    while not rospy.is_shutdown():
        rate.sleep()
        rate.sleep()

        iiwa_group.set_named_target("iiwa_Pose1")
        plan = iiwa_group.plan()
        iiwa_pose = moveit_msgs.msg.ExecuteTrajectoryGoal()
        iiwa_pose.trajectory = plan

        iiwa_client.send_goal(iiwa_pose)
        iiwa_client.wait_for_result()
        rospy.loginfo("Go to iiwa Pose 1")

        rate.sleep()
        rate.sleep()

        iiwa_group.set_named_target("iiwa_Pose2")
        plan = iiwa_group.plan()
        iiwa_pose = moveit_msgs.msg.ExecuteTrajectoryGoal()
        iiwa_pose.trajectory = plan

        iiwa_client.send_goal(iiwa_pose)
        iiwa_client.wait_for_result()
        rospy.loginfo("Go to iiwa Pose 2")

        rate.sleep()
        rate.sleep()

        iiwa_group.set_named_target("iiwa_Pose3")
        plan = iiwa_group.plan()
        iiwa_pose = moveit_msgs.msg.ExecuteTrajectoryGoal()
        iiwa_pose.trajectory = plan

        iiwa_client.send_goal(iiwa_pose)
        iiwa_client.wait_for_result()
        rospy.loginfo("Go to iiwa Pose 3")

        rate.sleep()
        rate.sleep()

        iiwa_group.set_named_target("iiwa_Pose4")
        plan = iiwa_group.plan()
        iiwa_pose = moveit_msgs.msg.ExecuteTrajectoryGoal()
        iiwa_pose.trajectory = plan

        iiwa_client.send_goal(iiwa_pose)
        iiwa_client.wait_for_result()
        rospy.loginfo("Go to iiwa Pose 4")

        rate.sleep()
        rate.sleep()

        iiwa_group.set_named_target("iiwa_Pose5")
        plan = iiwa_group.plan()
        iiwa_pose = moveit_msgs.msg.ExecuteTrajectoryGoal()
        iiwa_pose.trajectory = plan

        iiwa_client.send_goal(iiwa_pose)
        iiwa_client.wait_for_result()
        rospy.loginfo("Go to iiwa Pose 5")

        rate.sleep()
        rate.sleep()

        iiwa_group.set_named_target("iiwa_Pose6")
        plan = iiwa_group.plan()
        iiwa_pose = moveit_msgs.msg.ExecuteTrajectoryGoal()
        iiwa_pose.trajectory = plan

        iiwa_client.send_goal(iiwa_pose)
        iiwa_client.wait_for_result()
        rospy.loginfo("Go to iiwa Pose 6")

        rate.sleep()
        rate.sleep()

        iiwa_group.set_named_target("iiwa_Pose7")
        plan = iiwa_group.plan()
        iiwa_pose = moveit_msgs.msg.ExecuteTrajectoryGoal()
        iiwa_pose.trajectory = plan

        iiwa_client.send_goal(iiwa_pose)
        iiwa_client.wait_for_result()
        rospy.loginfo("Go to iiwa Pose 7")


if __name__=='__main__':
    try:
        iiwa_pose_place()
    except rospy.ROSInterruptException:
        moveit_commander.roscpp_shutdown()
        pass
