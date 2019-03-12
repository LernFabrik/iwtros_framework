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

    ur5_group = moveit_commander.MoveGroupCommander("ur5_arm")
    iiwa_group = moveit_commander.MoveGroupCommander("iiwa_arm")
    panda_group = moveit_commander.MoveGroupCommander("panda_arm")

    ur5_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    ur5_client.wait_for_server()
    rospy.loginfo("Execution Trajectroy server is available for ur5")
    iiwa_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    iiwa_client.wait_for_server()
    rospy.loginfo("Execution Trajectroy server is available for iiwa")
    panda_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    panda_client.wait_for_server()
    rospy.loginfo("Execution Trajectroy server is available for Panda")

    ur5_group.set_named_target("UR5_Home")
    plan = ur5_group.plan()
    ur5_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    ur5_goal.trajectory = plan

    ur5_client.send_goal(ur5_goal)
    ur5_client.wait_for_result()
    rospy.loginfo("Go to UR5 Home")

    iiwa_group.set_named_target("IIWA_Home")
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

        panda_group.set_named_target("panda_Pick")
        plan = panda_group.plan()
        panda_pick = moveit_msgs.msg.ExecuteTrajectoryGoal()
        panda_pick.trajectory = plan

        panda_client.send_goal(panda_pick)
        panda_client.wait_for_result()
        rospy.loginfo("Go to Panda pick")

        rate.sleep()

        iiwa_group.set_named_target("iiwa_Place")
        plan = iiwa_group.plan()
        iiwa_place = moveit_msgs.msg.ExecuteTrajectoryGoal()
        iiwa_place.trajectory = plan
        iiwa_client.send_goal(iiwa_place)
        iiwa_client.wait_for_result()
        rospy.loginfo("Go to iiwa place")

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
        iiwa_pick_place()
    except rospy.ROSInterruptException:
        moveit_commander.roscpp_shutdown()
        pass
