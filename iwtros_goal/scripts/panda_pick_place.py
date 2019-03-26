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

def panda_pick_place():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_pick_plcae', anonymous=True)

    panda_group = moveit_commander.MoveGroupCommander("panda_arm")

    panda_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    panda_client.wait_for_server()
    rospy.loginfo("Execution Trajectroy server is available for Panda")

    rate = rospy.Rate(5.0)
    
    while not rospy.is_shutdown():
        rate.sleep()

        panda_group.set_named_target("panda_Pick")
        plan = panda_group.plan()
        panda_pick = moveit_msgs.msg.ExecuteTrajectoryGoal()
        panda_pick.trajectory = plan

        panda_client.send_goal(panda_pick)
        panda_client.wait_for_result()
        rospy.loginfo("Go to Panda pick")

        rate.sleep()

        panda_group.set_named_target("panda_Place")
        plan = panda_group.plan()
        panda_pick = moveit_msgs.msg.ExecuteTrajectoryGoal()
        panda_pick.trajectory = plan

        panda_client.send_goal(panda_pick)
        panda_client.wait_for_result()
        rospy.loginfo("Go to Panda place")


if __name__=='__main__':
    try:
        panda_pick_place()
    except rospy.ROSInterruptException:
        moveit_commander.roscpp_shutdown()
        pass
