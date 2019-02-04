#!/usr/bin/env python

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg
import math 



pc_pub = rospy.Publisher("laserPointCloud", PointCloud2, queue_size=10)
lp = lg.LaserProjection()



def scanConverter(msg):
    pc2_msg = lp.projectLaser(msg)
    pc_pub.publish(pc2_msg)
    point_generator = pc2.read_points(pc2_msg)
    sum = 1.0
    num = 0
    for Point in point_generator:
        if not math.isnan(Point[2]):
            sum += Point[2]
            num += 1
    
    print(str(sum/num))
    piont_list = pc2.read_points_list(pc2_msg)
    print(piont_list[len(piont_list)/2].x)



if __name__ == "__main__":
    rospy.init_node("laserScaner_PointCloud2", anonymous=False)
    rospy.Subscriber("/scan", LaserScan, scanConverter, queue_size=1 )
    rospy.spin()
