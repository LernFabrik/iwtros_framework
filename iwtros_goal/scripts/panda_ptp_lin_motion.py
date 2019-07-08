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
_DEFAULT_PLANNING_GROUP = "panda_arm"
PLANNING_GROUP_NAME = "panda_arm"

def start_programme():
    rospy.loginfo("Start the program")

    start_pose = [0.45, -0.054, 0.308, -3.127, -0.014, -0.798]
    pick_pose = Pose(position=Point(0.45, -0.05, 0.3), orientation=from_euler(math.radians(-180), math.radians(180), math.radians(45)))
    Place_pose = Pose(position=Point(0.2, -0.3, 0.25), orientation=from_euler(math.radians(-180), math.radians(180), math.radians(0)))
    #Other_pose = Pose(position=Point(), orientation=from_euler(math.radians(0), math.radians(180), math.radians(90)))

    # move to start point joint values to avoid random trajectory
    r.move(Ptp(goal=pick_pose,  vel_scale=0.2, acc_scale=0.1, reference_frame="panda_link0", planning_group="panda_arm", target_link="panda_link8"))
    rospy.loginfo("Start pick and place loop....--->")
    while not rospy.is_shutdown():
        #Pick pose
        rospy.loginfo("move the robot to Place position")
        r.move(Ptp(goal=Place_pose,  vel_scale=0.2, acc_scale=0.2, reference_frame="panda_link0", planning_group="panda_arm", target_link="panda_link8"))
        Pick_and_Place()

        #Place pose
        rospy.loginfo("Move to Pick position") # log
        r.move(Ptp(goal=pick_pose,  vel_scale=0.2, acc_scale=0.2, reference_frame="panda_link0", planning_group="panda_arm", target_link="panda_link8"))
        rospy.loginfo("Pick movement") # log
        Pick_and_Place()


def Pick_and_Place():
    # the position is given relative to the TCP.
    r.move(Lin(goal=Pose(position=Point(0, 0, 0.1)), vel_scale=0.05, acc_scale=0.01, reference_frame="panda_link8", planning_group="panda_arm", target_link="panda_link8"))
    rospy.loginfo("Open/Close the gripper") # log
    rospy.sleep(0.2)    # pick or Place the PNOZ (close or open the gripper)
    r.move(Lin(goal=Pose(position=Point(0, 0, -0.1)), vel_scale=0.05, acc_scale=0.01, reference_frame="panda_link8", planning_group="panda_arm", target_link="panda_link8"))


if __name__ == "__main__":
    rospy.init_node("panda_pick_place_ptp_lin_node")

    r = Robot(__REQUIRED_API_VERSION__)
    start_programme()