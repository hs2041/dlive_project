#!/usr/bin/env python

import rospy
from basics.srv import wordcount, wordcountResponse

def count_words(request):

	return wordcountResponse(len(request.words.split()))


rospy.init_node('service_server')

service = rospy.Service('word_count', wordcount, count_words)

rospy.spin()