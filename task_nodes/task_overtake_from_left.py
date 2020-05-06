#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from decision_basis_msgs.msgs import OvertakeFromLeftBasis

def callback_todo(data_todo):
	pass

def listener():
	rospy.init_node("task_overtake_from_left", anonymous=True)
	# subscribers

	rospy.Subscriber("", type_todo, callback_todo, queue_size=1)

	# publishers
	pub_overtake_from_left_basis = rospy.Publisher("/decision_basis/overtake_from_left", OvertakeFromLeftBasis, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



