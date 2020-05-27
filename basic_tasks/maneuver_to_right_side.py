#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from basic_task_msgs import ManeuverToRightSideDecisionBasis 
from decision_msgs import Decision
from gridmap_msg import GridMap

def callback_todo(data_todo):
	pass

def listener():
	rospy.init_node("maneuver_to_right_side", anonymous=True)
	# subscribers

	rospy.Subscriber("/grid_map", GridMap, callback_todo, queue_size=1)

	# publishers
	pub_maneuver_to_right_side_decision_basis = rospy.Publisher("/maneuver_to_right_side/decision_basis", ManeuverToRightSideDecisionBasis, queue_size=1)
	pub_maneuver_to_right_side_decision = rospy.Publisher("/maneuver_to_right_side/decision", Decision, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



