#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float64

safe_distance = 1.0
brake_alert = 100
obstacle_dist = 1.0
safe_alert = 0
obst_dist = rospy.Publisher('obst_dist', Float64, queue_size=1)

def pol2cart(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return(x, y)

def scan_callback(data, obst_alert):
	global obstacle_dist
	global safe_distance
	global brake_alert
	global safe_alert
	global obst_dist

	obstacle_dist = min(data.ranges)
	#print("Obstacle Distance = ", obstacle_dist)
	obst_dist.publish(obstacle_dist)
	if obstacle_dist > safe_distance:
		safe_alert = int((obstacle_dist-safe_distance)*100/safe_distance)
		if safe_alert > 100:
			brake_alert = 0
		else:
			brake_alert = 100 - safe_alert
	else:
		brake_alert = 100
		
	obst_alert.publish(brake_alert)

rospy.init_node('obstacle_r2000')
obst_alert = rospy.Publisher('obst_alert', Int32, queue_size=1)
scan_sub = rospy.Subscriber('/r2000_scan', LaserScan, scan_callback, obst_alert)
rospy.spin()
