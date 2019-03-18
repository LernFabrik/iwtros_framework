#!/usr/bin/env python

#################################################################################
# Author: Vishnuprasad Prachandabhanu
#
##
## To use the python interface to move_group, import the moveit_commander
## module.  We also import rospy and some messages that we will use.

import copy
import sys

import actionlib
import geometry_msgs
import moveit_commander
import moveit_msgs.msg
import rospy


def getPosePoints(key):
  choices = {1: (-0.30, -0.4, 0.150), 
            2: (0.30, -0.40, 0.150), 
            3: (0.30, 0.40, 0.150), 
            4: (-0.30, 0.40, 0.150), 
            5: (0, 0.40, 0.2), 
            6: (0.30, 0, 0.2), 
            7: (0, 0.40, 0.2)}
  result = choices.get(key, ('default1', 'default2', 'default3', 'default4', 'default5', 'default6', 'default7'))
  return result


def simple_pick_place():
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('simple_movement',
                  anonymous=True)
  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group refers to the joints of
  ## IIWA7. This interface can be used to plan and execute motions on IIWA7.
  iiwa_group = moveit_commander.MoveGroupCommander("manipulator")
  ## Action clients to the ExecuteTrajectory action server.
  iiwa_client = actionlib.SimpleActionClient('execute_trajectory',
                                            moveit_msgs.msg.ExecuteTrajectoryAction)
  iiwa_client.wait_for_server()
  rospy.loginfo('Execute Trajectory server is available for IIWA7')
  ## Set a named joint configuration as the goal to plan for a move group.
  ## Named joint configurations are the robot poses defined via MoveIt! Setup Assistant.
  rate = rospy.Rate(20)
  loopCounter = 1
  while not rospy.is_shutdown():
    #  ## Cartesian Paths
    # ## ^^^^^^^^^^^^^^^
    # ## You can plan a cartesian path directly by specifying a list of waypoints
    # ## for the end-effector to go through.
    waypoints = []
    # start with the current pose
    current_pose = iiwa_group.get_current_pose()
    rospy.sleep(0.5)
    current_pose = iiwa_group.get_current_pose()
    rospy.loginfo("Curent Position x = %f, y = %f, z = %f" % (current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z))
    rospy.loginfo("Curent orientation x = %f, y = %f, z = %f, w = %f" % (current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w))
    
    ## create linear offsets to the current pose
    new_eef_pose = geometry_msgs.msg.Pose()

    for cout in range(1, 8):
      # Get pose values
      pose = getPosePoints(cout)
      new_eef_pose.position.x = pose[0]
      new_eef_pose.position.y = pose[1]
      new_eef_pose.position.z = pose[2]
      # Retain orientation of the current pose.
      new_eef_pose.orientation = copy.deepcopy(current_pose.pose.orientation)
      waypoints.append(new_eef_pose)

    waypoints.append(current_pose.pose)
    ## We want the cartesian path to be interpolated at a resolution of 1 cm
    ## which is why we will specify 0.01 as the eef_step in cartesian
    ## translation.  We will specify the jump threshold as 0.0, effectively
    ## disabling it.
    fraction = 0.0
    for count_cartesian_path in range(0,3):
      if fraction < 1.0:
        (plan_cartesian, fraction) = iiwa_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold
      else:
        break

    robot1_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
    robot1_goal.trajectory = plan_cartesian

    iiwa_client.send_goal(robot1_goal)
    iiwa_client.wait_for_result()
    loopCounter = loopCounter + 1
    if loopCounter == 2:
      loopCounter = 1
    rate.sleep()


if __name__=='__main__':
  try:
    simple_pick_place()
  except rospy.ROSInterruptException:
    moveit_commander.roscpp_shutdown()
    pass
