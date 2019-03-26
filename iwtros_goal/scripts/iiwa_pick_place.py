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


def iiwa_pick_place():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('iiwa_pick_plcae', anonymous=True)

    iiwa_group = moveit_commander.MoveGroupCommander("iiwa_arm")

    iiwa_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    iiwa_client.wait_for_server()
    rospy.loginfo("Execution Trajectroy server is available for iiwa")


    iiwa_group.set_named_target("iiwa_Home")
    plan = iiwa_group.plan()
    iiwa_go_to_home = moveit_msgs.msg.ExecuteTrajectoryGoal()
    iiwa_go_to_home.trajectory = plan

    iiwa_client.send_goal(iiwa_go_to_home)
    iiwa_client.wait_for_result()
    rospy.loginfo("Go to IIWA Home")
    
    rate = rospy.Rate(5.0)
    
    while not rospy.is_shutdown():
        rate.sleep()

        iiwa_group.set_named_target("iiwa_Pick")
        plan = iiwa_group.plan()
        iiwa_pick = moveit_msgs.msg.ExecuteTrajectoryGoal()
        iiwa_pick.trajectory = plan

        iiwa_client.send_goal(iiwa_pick)
        iiwa_client.wait_for_result()
        rospy.loginfo("Go to iiwa pick")

        rate.sleep()

        iiwa_group.set_named_target("iiwa_Place")
        plan = iiwa_group.plan()
        iiwa_place = moveit_msgs.msg.ExecuteTrajectoryGoal()
        iiwa_place.trajectory = plan
        iiwa_client.send_goal(iiwa_place)
        iiwa_client.wait_for_result()
        rospy.loginfo("Go to iiwa place")



if __name__=='__main__':
    try:
        iiwa_pick_place()
    except rospy.ROSInterruptException:
        moveit_commander.roscpp_shutdown()
        pass
