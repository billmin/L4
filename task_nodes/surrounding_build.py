#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from decision_basis_msgs.msgs import SurroundingBuild

def callback_todo(data_todo):
	pass

def listener():
	rospy.init_node("surrounding_build", anonymous=True)
	# subscribers

	rospy.Subscriber("", type_todo, callback_todo, queue_size=1)

	# publishers
	pub_surrounding_build = rospy.Publisher("/decision_basis/surrounding_build", SurroundingBuild, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



