#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose
import time

way_points = np.loadtxt('refPoses.dat')
way_points = np.asarray(way_points)
n = np.asarray(way_points.shape)

i = 0
pub = rospy.Publisher('way_point',Pose,queue_size=10)
rospy.init_node('talker',anonymous=True)
rate = rospy.Rate(1)
point = Pose()

while i < n[0]:
                    
    point.position.x = way_points[i,0]
    point.position.y = way_points[i,1]
    point.position.z = 0

    point.orientation.x = 0
    point.orientation.y = 0
    point.orientation.z = 1
    point.orientation.w = way_points[i,2]
    
    pub.publish(point)
    i = i + 1
    

