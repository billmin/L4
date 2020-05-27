#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from navigation_msg import NavigationInfo
from gridmap_msg import GridMap
from decision_msg import Decision

# published messages
from control_msgs import XControlStrategy, YControlStrategy

def callback_navigation(data_navi_info):
	global navi_info
	navi_info = data_navi_info

def callback_grid_map(data_grid_map):
	global grid_map 
	grid_map = data_grid_map


def listener():
	rospy.init_node("decision_maker", anonymous=True)
	# subscribers
	rospy.Subscriber("/navigation", NavigationInfo, callback_navigation, queue_size=1)
	rospy.Subscriber("/grid_map", GridMap, callback_grid_map, queue_size=1)

	# publishers
	pub_steering_strategy = rospy.Publisher("/steering_strategy", XControlStrategy, queue_size=1)
	pub_longitudinal_strategy = rospy.Publisher("/longitudinal_strategy", YControlStrategy, queue_size=1)

	rate = rospy.Rate(10)

	# navigation info
	global navi_info
	navi_info = NavigationInfo()
	# grid map
	global grid_map
	grid_map = GridMap()

	while not rospy.is_shutdown():

		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



