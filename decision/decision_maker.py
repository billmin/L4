#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os
from global_vars import *
from transitions import Machine

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

	# navigation
	navigator = Navigation()
	navi_fsm = Machine(model=navigator, states=navi_states, transitions=navi_state_transitions, initial=navi_states[0])
	# maneuver
	maneuver = Maneuver()
	maneuver_fsm = Machine(model=maneuver, states=maneuver_states, transitions=maneuver_state_transition, initial=maneuver_states[0])

	while not rospy.is_shutdown():
		# get navigation command
		navi_command_code = navi_info.navi_command
		navi_warning_code = navi_info.navi_warning
		# state transition according to navi command
		getattr(navigator, navi_commands[navi_command_code])()
				

		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



