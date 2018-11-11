#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64

angular_state = 0
angular_setpoint = 0
a = 3
kp = 1
ki = 1
kd = 1
time_prev = 0
error_prev = 0
integrated_error = 0

def state_callback(data):
	global a 
	a += data.data

	
if __name__ == '__main__':

	global a

	rospy.init_node('pid_steer_state')

	sub_state = rospy.Publisher('state_steer', Float64, queue_size = 1)	
	control_state = rospy.Subscriber('control_effort1', Float64, state_callback)

	
	while not rospy.is_shutdown():
		
		sub_state.publish(a)

	rospy.spin()
