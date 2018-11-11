#!/usr/bin/env python

import rospy
from basics.srv import squarenumber, squarenumberResponse

def count_words(request):

	return squarenumberResponse(request.number * request.number)


rospy.init_node('service_server')

service = rospy.Service('square_number', squarenumber, count_words)

rospy.spin()