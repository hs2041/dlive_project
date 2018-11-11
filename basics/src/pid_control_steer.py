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

def setpoint_callback(data):
	global angular_setpoint 

	angular_setpoint = data.data

def state_callback(data):
	global angular_state 
	
	angular_state = data.data

def pid_on():
	global kp,ki,kd,angular_state,angular_setpoint,time_prev,error_prev, integrated_error

	time_cur = time.time()
	dt = time_cur - time_prev

	if dt == 0:
		return 0

	error_cur = angular_setpoint - angular_state
	integrated_error += error_cur*dt
	derivative_error = (error_cur - error_prev)/dt

	error_prev = error_cur
	time_prev = time_cur

	control_effort = kp*error_cur + ki*integrated_error + kd*derivative_error
	print(control_effort)
	pub_control.publish(control_effort)

	
if __name__ == '__main__':

	rospy.init_node('pid_steer')

	sub_setpoint = rospy.Subscriber('setpoint_steer', Float64, setpoint_callback)
	sub_state = rospy.Subscriber('state_steer', Float64, state_callback)

	pub_control = rospy.Publisher('control_effort1',Float64, queue_size = 1)

	while not rospy.is_shutdown():
		print("1")
		pid_on()
		time.sleep(1)

	rospy.spin()
