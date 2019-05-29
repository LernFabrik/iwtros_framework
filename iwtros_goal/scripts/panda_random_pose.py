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

def panda_pose_place():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_random_motion', anonymous=True)

    panda_group = moveit_commander.MoveGroupCommander("panda_arm")
    panda_group.set_max_velocity_scaling_factor(0.15)
    panda_group.set_max_acceleration_scaling_factor(0.3)

    panda_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    panda_client.wait_for_server()
    rospy.loginfo("Execution Trajectroy server is available for panda")

    rate = rospy.Rate(0.5)
    
    while not rospy.is_shutdown():
        rate.sleep()
        rate.sleep()

        panda_group.set_named_target("panda_Pose1")
        plan = panda_group.plan()
        panda_pose = moveit_msgs.msg.ExecuteTrajectoryGoal()
        panda_pose.trajectory = plan

        panda_client.send_goal(panda_pose)
        panda_client.wait_for_result()
        rospy.loginfo("Go to panda Pose 1")

        rate.sleep()
        rate.sleep()

        panda_group.set_named_target("panda_Pose2")
        plan = panda_group.plan()
        panda_pose = moveit_msgs.msg.ExecuteTrajectoryGoal()
        panda_pose.trajectory = plan

        panda_client.send_goal(panda_pose)
        panda_client.wait_for_result()
        rospy.loginfo("Go to panda Pose 2")

        rate.sleep()
        rate.sleep()

        panda_group.set_named_target("panda_Pose3")
        plan = panda_group.plan()
        panda_pose = moveit_msgs.msg.ExecuteTrajectoryGoal()
        panda_pose.trajectory = plan

        panda_client.send_goal(panda_pose)
        panda_client.wait_for_result()
        rospy.loginfo("Go to panda Pose 3")


if __name__=='__main__':
    try:
        panda_pose_place()
    except rospy.ROSInterruptException:
        moveit_commander.roscpp_shutdown()
        pass
