#!/usr/bin/env python
#This file created and modified for the purpose of 
# ROS-framework for LernFabrik work place simulation 
# Author: Vishnuprasad Prachandabhanu

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from std_msgs.msg import Bool

start = False


#def callback(data):
 #   start = data.data
  #  rospy.loginfo("got the command %b", start)



def ur5_pick_place():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_pick_place', anonymous=True)
    #rospy.Subscriber("startUr5", Bool, callback)
    
    ur5_group = moveit_commander.MoveGroupCommander("ur5_arm")
    ur5_group.set_max_acceleration_scaling_factor(1.0)
    ur5_group.set_max_velocity_scaling_factor(1.0)

    ur5_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    ur5_client.wait_for_server()
    rospy.loginfo("Execution Trajectroy server is available for ur5")
    
    rate = rospy.Rate(5.0)
    
    while not rospy.is_shutdown():
        #rospy.loginfo("Waiting for the command to start")
        #if start == True:
        rate.sleep()
        
        ur5_group.set_named_target("ur5_Pick")
        plan = ur5_group.plan()
        ur5_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        ur5_goal.trajectory = plan

        ur5_client.send_goal(ur5_goal)
        ur5_client.wait_for_result()
        rospy.loginfo("Go to UR5 Pick")

        rate.sleep()

        ur5_group.set_named_target("ur5_Place")
        plan = ur5_group.plan()
        ur5_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        ur5_goal.trajectory = plan

        ur5_client.send_goal(ur5_goal)
        ur5_client.wait_for_result()
        rospy.loginfo("Go to UR5 Place")

            #rospy.spin()
        #rate.sleep()
        #rospy.spin()

   


if __name__=='__main__':
    try:
        ur5_pick_place()
    except rospy.ROSInterruptException:
        moveit_commander.roscpp_shutdown()
        pass
