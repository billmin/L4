#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from basic_task_msgs import GeneralAdaptiveCruiseDecisionBasis
from gridmap_msg import GridMap

# published messages
from control_msgs import XControlStrategy, YControlStrategy


def callback_todo(data_todo):
	pass

def callback_active_task(active_task_id):
	global task_id
	task_id = active_task_id

def listener():
	rospy.init_node("general_adaptive_cruise", anonymous=True)
	# subscribers

	rospy.Subscriber("/grid_map", GridMap, callback_todo, queue_size=1)
	rospy.Subscriber("/active_task_id", Int8, callback_active_task, queue_size=1)

	# publishers
	pub_general_adaptive_cruise_decision_basis = rospy.Publisher("/general_adaptive_cruise/decision_basis", GeneralAdaptiveCruiseDecisionBasis, queue_size=1)
	pub_general_adaptive_cruise_decision = rospy.Publisher("/general_adaptive_cruise/decision", Decision, queue_size=1)

	# controls
	pub_steering_strategy = rospy.Publisher("/steering_strategy", XControlStrategy, queue_size=1)
	pub_longitudinal_strategy = rospy.Publisher("/longitudinal_strategy", YControlStrategy, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



