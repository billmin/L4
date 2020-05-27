#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from control_msgs.msgs import YControlStrategy


def callback_longitudinal_strategy(data_lng_strategy):
	global longitudinal_strategy
	longitudinal_strategy = data_lng_strategy

def listener():
	rospy.init_node("ycontroller", anonymous=True)
	# subscribers
	rospy.Subscriber("/longitudinal_strategy", YControlStrategy, callback_longitudinal_strategy, queue_size=1)

	# publishers
	pub_torque = rospy.Publisher("/torque", Float32, queue_size=1)
	pub_brake = rospy.Publisher("/brake", Float32, queue_size=1)

	rate = rospy.Rate(10)

	global longitudinal_strategy
	longitudinal_strategy = YControlStrategy()


	while not rospy.is_shutdown():
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



