#!/usr/bin/env python

"""
This programme uses the PTP, LIN and CIRC industrial motion developed by the
Pilz Robotics GmbH
https://github.com/PilzDE/pilz_industrial_motion/tree/melodic-devel/pilz_robot_programming
"""

from geometry_msgs.msg import Pose, Point
from pilz_robot_programming import *
import math
import rospy
import moveit_commander
import actionlib
import moveit_ros_planning_interface
import moveit_msgs.msg
import copy
import sys

__REQUIRED_API_VERSION__ = "1" 
__ROBOT_VELOCITY__ = 0.1
__ROBOT_VELOCITY_LIN__ = 0.07
__ROBOT_ACCELERATION_LIN__ = 0.08
_DEFAULT_PLANNING_GROUP = "panda_arm"
PLANNING_GROUP_NAME = "panda_arm"

def start_programme():
    hand_group = moveit_commander.MoveGroupCommander("panda_hand")
    hand_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
    hand_client.wait_for_server()
    rospy.loginfo("Initialized the gripper and ")
    rospy.loginfo("Start the program")

    start_pose = [0.45, -0.054, 0.308, -3.127, -0.014, -0.798]
    pick_pose = Pose(position=Point(0.66, 0.25, 0.3), orientation=from_euler(math.radians(0), math.radians(180), math.radians(135)))
    Place_pose = Pose(position=Point(0.50, -0.25, 0.3), orientation=from_euler(math.radians(0), math.radians(180), math.radians(135)))
    Other_pose = Pose(position=Point(0.66, 0, 0.3), orientation=from_euler(math.radians(0), math.radians(90), math.radians(0)))

    # move to start point joint values to avoid random trajectory
    r.move(Ptp(goal=pick_pose, vel_scale=0.2, acc_scale=0.2, reference_frame="panda_link0", planning_group="panda_arm", target_link="panda_link8"))
    rospy.loginfo("Start pick and place loop....--->")
    while not rospy.is_shutdown():
        #Pick pose
        hand_group.(0.2, end_effector_link="panda_rightfinger")
        hand_group.go()
        rospy.loginfo("move the robot to Place position")
        r.move(Ptp(goal=Place_pose,  vel_scale=0.2, acc_scale=0.2, reference_frame="panda_link0", planning_group="panda_arm", target_link="panda_link8"))
        Pick_and_Place()
        #r.move(Ptp(goal=Other_pose,  vel_scale=0.2, acc_scale=0.2, reference_frame="panda_link0", planning_group="panda_arm", target_link="panda_link8"))

        #Place pose
        rospy.loginfo("Move to Pick position") # log
        r.move(Ptp(goal=pick_pose,  vel_scale=0.2, acc_scale=0.2, reference_frame="panda_link0", planning_group="panda_arm", target_link="panda_link8"))
        rospy.loginfo("Pick movement") # log
        Pick_and_Place()
        # try:
        #         r.move(sequence)
        # except RobotMoveFailed:
        #         rospy.logerr("<------failed to move----->")
                


def Pick_and_Place():
        # the position is given relative to the TCP.
        #r.move(Gripper(goal=0.02, planning_group="panda_hand"))
        r.move(Lin(goal=Pose(position=Point(0, 0, 0.1)), vel_scale=0.03, acc_scale=0.03, reference_frame="panda_link8", planning_group="panda_arm", target_link="panda_link8"))
        rospy.loginfo("Open/Close the gripper") # log
        rospy.sleep(0.2)    # pick or Place the PNOZ (close or open the gripper)
        r.move(Lin(goal=Pose(position=Point(0, 0, -0.1)), vel_scale=0.03, acc_scale=0.03, reference_frame="panda_link8", planning_group="panda_arm", target_link="panda_link8"))


if __name__ == "__main__":
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("panda_pick_place_ptp_lin_node")

        r = Robot(__REQUIRED_API_VERSION__)
        sequence = Sequence()
        start_programme()