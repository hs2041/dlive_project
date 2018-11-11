#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
from e2o.msg import e2o_ctrl, e2o_status

#car_control = e2o_ctrl()
ackermann_ctrl = AckermannDrive()
last_speed = 0
a1		# if input acceleration is a then top speed that can be reached is a*a1
a2		# increment speed by a2*a
a3		# decrement speed by a3*a
b1		# decrement speed by b1*brake %
c1		# when gear is neutral, decrement speed by c1

i = 0 # i is used so that no ackermann_ctrl is published until we get the first e2o_ctrl msg

def car_callback(car_control):

	global ackermann_ctrl,last_speed,i
	i = 1
	ackermann_ctrl.steering_angle = car_control.Steer * pi/180

	if abs(ackermann_ctrl.steering_angle) > 40*pi/180:
		if ackermann_ctrl.steering_angle > 0:
			ackermann_ctrl.steering_angle = 40*pi/180
		else:
			ackermann_ctrl.steering_angle = -40*pi/180

	#at any point of time one of the brakes and acceleration is equal to zero
	if car_control.Accel > 0 and car_control.RNDB != 'N':
		
			if abs(last_speed - a1*car_control.Accel) < a2*car_control.Accel:
				speed = a1*car_control.Accel

			if last_speed > a1*car_control.Accel:
				speed = last_speed - a3*car_control.Accel
			elif last_speed < a1*car_control.Accel:
				speed = last_speed + a2*car_control.Accel
			else:
				speed = last_speed

	
	elif car_control.Brake > 0:

		speed = last_speed - b1*car_control.Brake
		if speed < 0:
			speed = 0

	elif car_control.RNDB == 'N' or (car_control.Accel == 0 and car_control.Brake == 0) :
		speed = last_speed - c1
		if speed < 0:
			speed = 0

	last_speed = speed
	
	if car_control.RNDB == 'R':
		speed = speed * (-1)
	
	ackermann_ctrl.speed = speed




if __name__ == '__main__':

	global ackermann_ctrl

	rospy.init_node('can_receiver')
	ctrl_pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=1)
	rospy.Subscriber('e2octrl', e2o_ctrl2, car_callback)
	rate = rospy.Rate(10)


	while not rospy.is_shutdown():
		if i != 0:
			ctrl_pub.publish(ackermann_ctrl)
		rate.sleep()

	rospy.spin()