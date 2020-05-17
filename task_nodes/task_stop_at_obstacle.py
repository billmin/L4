#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from multi_task_msgs import StopAtObstacleBasis
from decision_basis_msgs import Decision

def callback_todo(data_todo):
	pass

def listener():
	rospy.init_node("task_stop_at_obstacle", anonymous=True)
	# subscribers

	rospy.Subscriber("", XControlStrategy, callback_steering_strategy, queue_size=1)

	# publishers
	pub_stop_at_obstacle_basis = rospy.Publisher("/multi_task/stop_at_obstacle", StopAtObstacleBasis, queue_size=1)
	pub_stop_at_obstacle_decision = rospy.Publisher("/stop_at_obstacle/decision", Decision, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



