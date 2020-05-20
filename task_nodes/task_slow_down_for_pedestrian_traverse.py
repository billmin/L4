#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from multi_task_msgs import SlowDownForPedestrianTraverseBasis
from decision_basis_msgs import Decision

def callback_todo(data_todo):
	pass

def listener():
	rospy.init_node("task_slow_down_for_pedestrian_traverse", anonymous=True)
	# subscribers

	rospy.Subscriber("", type_todo, callback_todo, queue_size=1)

	# publishers
	pub_slow_down_for_pedestrian_traverse_basis = rospy.Publisher("/multi_task/slow_down_for_pedestrian_traverse", SlowDownForPedestrianTraverseBasis, queue_size=1)
	pub_slow_down_for_pedestrian_traverse_decision = rospy.Publisher("/slow_down_for_pedestrian_traverse/decision", Decision, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



