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


def ur5_home():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_home', anonymous=True)

    ur5_group = moveit_commander.MoveGroupCommander("ur5_arm")

    ur5_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    ur5_client.wait_for_server()
    rospy.loginfo("Execution Trajectroy server is available for iiwa")


    ur5_group.set_named_target("ur5_Home")
    plan = ur5_group.plan()
    iiwa_go_to_home = moveit_msgs.msg.ExecuteTrajectoryGoal()
    iiwa_go_to_home.trajectory = plan

    ur5_client.send_goal(iiwa_go_to_home)
    ur5_client.wait_for_result()
    rospy.loginfo("Go to UR5 Home")



if __name__=='__main__':
    try:
        ur5_home()
    except rospy.ROSInterruptException:
        moveit_commander.roscpp_shutdown()
        pass
