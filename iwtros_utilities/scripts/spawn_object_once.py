#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from gazebo_msgs.msg import ModelStates

def model_state_cb(data):
    if any("object" in idx for idx in data.name):
        stop_spawn = rospy.ServiceProxy('stop_spawn', Empty)
        stop_spawn()
        rospy.signal_shutdown('One obeject is successfully spawned')


if __name__ == "__main__":
    rospy.init_node('stop_spawning_object', anonymous= False)
    rospy.Subscriber('gazebo/model_states', ModelStates, model_state_cb)
    rospy.spin()