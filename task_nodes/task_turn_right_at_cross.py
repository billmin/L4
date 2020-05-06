#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from decision_basis_msgs.msgs import TurnRightAtCrossBasis

def callback_todo(data_todo):
	pass

def listener():
	rospy.init_node("task_turn_right_at_cross", anonymous=True)
	# subscribers

	rospy.Subscriber("", type_todo, callback_todo, queue_size=1)

	# publishers
	pub_turn_right_at_cross_basis = rospy.Publisher("/decision_basis/turn_right_at_cross", TurnRightAtCrossBasis, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



