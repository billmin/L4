#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from decision_basis_msgs.msgs import TemporaryStopByRoadsideBasis

def callback_todo(data_todo):
	pass

def listener():
	rospy.init_node("task_temporary_stop_by_roadside", anonymous=True)
	# subscribers

	rospy.Subscriber("", type_todo, callback_todo, queue_size=1)

	# publishers
	pub_temporary_stop_by_roadside_basis = rospy.Publisher("/decision_basis/temporary_stop_by_roadside", TemporaryStopByRoadsideBasis, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



