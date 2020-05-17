#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from multi_task_msgs import TurnLeftAtCrossBasis
from decision_basis_msgs import Decision

def callback_todo(data_todo):
	pass

def listener():
	rospy.init_node("task_turn_left_at_cross", anonymous=True)
	# subscribers

	rospy.Subscriber("", type_todo, callback_todo, queue_size=1)

	# publishers
	pub_turn_left_at_cross_basis = rospy.Publisher("/multi_task/turn_left_at_cross", TurnLeftAtCrossBasis, queue_size=1)
	pub_turn_left_at_cross_decision = rospy.Publisher("/turn_left_at_cross/decision", Decision, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



