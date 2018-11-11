#!/usr/bin/env python
import rospy
from basics.msg import msg_template

rospy.init_node('message_publisher')
pub = rospy.Publisher('message_template', msg_template, queue_size = 2	)

rate = rospy.Rate(2)
msg = msg_template()
i = 1
a = 'i'
msg.data1 = 0
msg.data2 = False
msg.data3 = 0
msg.data4 = 0
msg.data5 = a

while not rospy.is_shutdown():
	msg.data3 = i
	i = i+1
	msg.data5 += a
	pub.publish(msg)


	rate.sleep()
rospy.spin()