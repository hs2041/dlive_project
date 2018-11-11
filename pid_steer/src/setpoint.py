#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64

angular_state = 0
angular_setpoint = 0

kp = 1
ki = 1
kd = 1
time_prev = 0
error_prev = 0
integrated_error = 0

	
if __name__ == '__main__':

	rospy.init_node('pid_steer_setpoint')

	sub_setpoint = rospy.Publisher('setpoint_steer', Float64, queue_size = 1)
	
	a = 4 
	while not rospy.is_shutdown():
		
		sub_setpoint.publish(a)

	rospy.spin()
