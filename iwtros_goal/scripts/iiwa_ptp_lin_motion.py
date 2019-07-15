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

__REQUIRED_API_VERSION__ = "1" 
__ROBOT_VELOCITY__ = 0.1
__ROBOT_VELOCITY_LIN__ = 0.07
__ROBOT_ACCELERATION_LIN__ = 0.08


def start_programme():
    rospy.loginfo("Start the program")

    start_pose = [0.45, -0.054, 0.308, -3.127, -0.014, -0.798]
    pick_pose = Pose(position=Point(0.60, 0.25, 0.5), orientation=from_euler(math.radians(180), math.radians(0), math.radians(0)))
    Place_pose = Pose(position=Point(0.6, -0.25, 0.5), orientation=from_euler(math.radians(180), math.radians(0), math.radians(0)))
    Other_pose = Pose(position=Point(0.66, 0, 0.3), orientation=from_euler(math.radians(0), math.radians(90), math.radians(0)))

    # move to start point joint values to avoid random trajectory
    r.move(Ptp(goal=pick_pose, vel_scale=0.2, acc_scale=0.2, reference_frame="iiwa_link_0", planning_group="iiwa_arm", target_link="iiwa_link_ee"))
    rospy.loginfo("Start pick and place loop....--->")
    while not rospy.is_shutdown():
        #Pick pose
        rospy.loginfo("move the robot to Place position")
        r.move(Ptp(goal=Place_pose,  vel_scale=0.2, acc_scale=0.2, reference_frame="iiwa_link_0", planning_group="iiwa_arm", target_link="iiwa_link_ee"))
        Pick_and_Place()
        #r.move(Ptp(goal=Other_pose,  vel_scale=0.2, acc_scale=0.2, reference_frame="iiwa_link_0", planning_group="iiwa_arm", target_link="iiwa_link_ee"))

        #Place pose
        rospy.loginfo("Move to Pick position") # log
        r.move(Ptp(goal=pick_pose,  vel_scale=0.2, acc_scale=0.2, reference_frame="iiwa_link_0", planning_group="iiwa_arm", target_link="iiwa_link_ee"))
        rospy.loginfo("Pick movement") # log
        Pick_and_Place()
        # try:
        #         r.move(sequence)
        # except RobotMoveFailed:
        #         rospy.logerr("<------failed to move----->")
                


def Pick_and_Place():
    # the position is given relative to the TCP.
    r.move(Lin(goal=Pose(position=Point(0, 0, 0.1)), vel_scale=0.03, acc_scale=0.03, reference_frame="iiwa_link_ee", planning_group="iiwa_arm", target_link="iiwa_link_ee"))
    rospy.loginfo("Open/Close the gripper") # log
    rospy.sleep(0.2)    # pick or Place the PNOZ (close or open the gripper)
    r.move(Lin(goal=Pose(position=Point(0, 0, -0.1)), vel_scale=0.03, acc_scale=0.03, reference_frame="iiwa_link_ee", planning_group="iiwa_arm", target_link="iiwa_link_ee"))


if __name__ == "__main__":
    rospy.init_node("iiwa_pick_place_ptp_lin_node")

    r = Robot(__REQUIRED_API_VERSION__)
    #sequence = Sequence()

    start_programme()