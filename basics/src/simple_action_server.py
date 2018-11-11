#! /usr/bin/env python
import rospy
import time
import actionlib

from basics.msg import TimerAction, TimerGoal, TimerResult

def do_timer(goal):# only the TimerGoal gets passed to do_timer
	start_time = time.time()
	time.sleep(goal.time_to_wait.to_sec())
	result = TimerResult()
	result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
	result.updates_sent = 8 #random no.
	server.set_succeeded(result)

rospy.init_node('timer_action_server')
server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)
#False disables autostarting of the server
server.start()
rospy.spin()