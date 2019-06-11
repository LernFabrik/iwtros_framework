#!/usr/bin/env python

import rospy
from wsg_50_common.srv import OpticalCMD
import struct

FMF_READ_FINGER_1 = 0x63
FMF_READ_FINGER_2 = 0x73

if __name__ == '__main__':

  rospy.init_node("force_test", anonymous=True)
  rospy.wait_for_service("/optical_weiss/fmf_cmd")

  fmf_cmd = rospy.ServiceProxy("/optical_weiss/fmf_cmd", OpticalCMD)
  while True:
    msg = OpticalCMD()
    msg.finger_idx = 1
    msg.cmd = FMF_READ_FINGER_2
    msg.tx_data = bytes()
    print("Sending fmf request")
    try:
      resp = fmf_cmd(msg.finger_idx, msg.cmd, msg.tx_data)
      print struct.unpack('<f',resp.rx_data[2:6])
    except rospy.ServiceException, e:
      print 'Service call failed: %s'%e
    rospy.sleep(1)
