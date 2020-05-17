#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from multi_task_msgs import CruiseAtHighSpeedBasis
from decision_basis_msgs import Decision

def callback_todo(data_todo):
	pass

def listener():
	rospy.init_node("task_cruise_at_high_speed", anonymous=True)
	# subscribers

	rospy.Subscriber("", type_todo, callback_todo, queue_size=1)

	# publishers
	pub_cruise_at_high_speed_basis = rospy.Publisher("/multi_task/cruise_at_high_speed", CruiseAtHighSpeedBasis, queue_size=1)
	pub_cruise_at_high_speed_decision = rospy.Publisher("/cruise_at_high_speed/decision")

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



