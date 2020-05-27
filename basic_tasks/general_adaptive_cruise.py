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
from decision_msgs import Decision
from gridmap_msg import GridMap

def callback_todo(data_todo):
	pass

def listener():
	rospy.init_node("general_adaptive_cruise", anonymous=True)
	# subscribers

	rospy.Subscriber("/grid_map", GridMap, callback_todo, queue_size=1)

	# publishers
	pub_general_adaptive_cruise_decision_basis = rospy.Publisher("/general_adaptive_cruise/decision_basis", GeneralAdaptiveCruiseDecisionBasis, queue_size=1)
	pub_general_adaptive_cruise_decision = rospy.Publisher("/general_adaptive_cruise/decision", Decision, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



