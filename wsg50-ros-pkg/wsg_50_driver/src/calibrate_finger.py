#!/usr/bin/env python

import rospy
from wsg_50_common.srv import OpticalCMD, Move
import numpy as np
import matplotlib.pyplot as plt

SPI_READ_ALL_CMD = 1
SPI_SET_REGISTER_CMD = 2
SPI_GET_REGISTER_CMD = 3
SPI_WRITE_REGISTER_CMD = 4

SAMPLES = 100
SURFACE_OFFSET = -3.5
move_cmd = None
optical_cmd = None

target_widths = [5,10,15,20,25,30,35,40]

def move_and_measure(width):
  move_msg = Move()
  move_msg.width = width
  move_msg.speed = 3.0
  
  try:
    resp = move_cmd(move_msg.width, move_msg.speed)
  except rospy.ServiceException, e:
    print "Service call failed: %s" %e
  
  rospy.sleep(1)
  
  optical_msg = OpticalCMD()
  optical_msg.finger_idx = 0
  optical_msg.cmd = SPI_READ_ALL_CMD
  optical_msg.tx_data = bytes(bytearray.fromhex('0101'))

  
  vals = []
  for i in xrange(SAMPLES):
    try:
      resp = optical_cmd(optical_msg.finger_idx, optical_msg.cmd, optical_msg.tx_data)
      vals.append(256*ord(resp.rx_data[2])+ord(resp.rx_data[3]))
    except rospy.ServiceException, e:
      print 'Service call failed: %s'%e
      
      
  val_sum = 0.0
  sq_val_sum = 0.0    
  for i in xrange(len(vals)):
    val_sum += vals[i]
    sq_val_sum += vals[i]**2
  mean = val_sum / len(vals)
  std_dev = (sq_val_sum/len(vals)-mean**2)**0.5
  
  return mean, std_dev
  

if __name__ == '__main__':
  
  rospy.init_node("calibrate_finger", anonymous=True)
  rospy.wait_for_service("/optical_weiss/optical_cmd")
  
  move_cmd = rospy.ServiceProxy("/optical_weiss/move", Move)
  optical_cmd = rospy.ServiceProxy("/optical_weiss/optical_cmd", OpticalCMD)
  msg = OpticalCMD()
  msg.finger_idx = 0
  msg.cmd = SPI_SET_REGISTER_CMD
  msg.tx_data = bytes(bytearray.fromhex('66'))
  
  try:
    resp = optical_cmd(msg.finger_idx, msg.cmd, msg.tx_data)
    print resp
  except rospy.ServiceException, e:
    print 'Service call failed: %s'%e
    
  x_vals = []
  means = []
  std_devs = []  
  for i in xrange(len(target_widths)):
    x_vals.append(target_widths[i]+SURFACE_OFFSET)
    mean, std_dev = move_and_measure(target_widths[i])
    means.append(mean)
    std_devs.append(std_dev)

  print x_vals
  print means
  print std_devs
  
  fig = plt.figure(1)
  plt.xlabel('Distance')
  plt.ylabel('Signal')
  plt.errorbar(x_vals, means,yerr=std_devs)
  plt.xlim(target_widths[0]+SURFACE_OFFSET-2, target_widths[-1]+SURFACE_OFFSET+2)
  plt.grid()
  plt.savefig('/tmp/calibration.png')
  plt.show()
