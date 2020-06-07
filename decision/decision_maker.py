#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os
from global_vars import *

# recieved messages
from navigation_msg import NavigationInfo
from gridmap_msg import GridMap
from decision_msg import Decision


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
	pub_active_task_id = rospy.Publisher("/active_task_id", Int8, queue_size=1)

	rate = rospy.Rate(50)

	# navigation info
	global navi_info
	navi_info = NavigationInfo()
	# grid map
	global grid_map
	grid_map = GridMap()

	# navigation related
	int navi_command_code, navi_warning_code
	# id of active task deployed
	active_task = BasicTask.GENERAL_ADAPTIVE_CRUISE.value

	while not rospy.is_shutdown():
		# get navigation command
		navi_command_code = navi_info.navi_command
		navi_warning_code = navi_info.navi_warning

		# ---- navigation level 
		if navi_command_code == NaviCommands.GO_STRAIGHT.value:

		elif navi_command_code == NaviCommands.TURN_LEFT.value:
		
		elif navi_command_code == NaviCommands.TURN_RIGHT.value:


		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



