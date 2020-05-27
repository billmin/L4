#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from control_msgs.msgs import XControlStrategy

def callback_steering_strategy(data_steering_strategy):
	global steering_strategy
	steering_strategy = data_steering_strategy


def listener():
	rospy.init_node("xcontroller", anonymous=True)
	# subscribers

	rospy.Subscriber("/steering_strategy", XControlStrategy, callback_steering_strategy, queue_size=1)

	# publishers
	pub_steering = rospy.Publisher("/control/steering_angle", Float32, queue_size=1)
    pub_steering_speed_limit = rospy.Publisher("/control/steer_speed_limit", Float32, queue_size=1)

	rate = rospy.Rate(10)

	#Float32MultiArray().data

	global steering_strategy
	steering_strategy = XControlStrategy()

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



