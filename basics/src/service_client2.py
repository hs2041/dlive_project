#!/usr/bin/env python

import rospy
import sys
from basics.srv import squarenumber

rospy.init_node('service_client')
rospy.wait_for_service('square_number')

word_counter = rospy.ServiceProxy('square_number', squarenumber)

words = input()
word_count = word_counter(words)

print'->',word_count.square
