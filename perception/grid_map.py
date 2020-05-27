#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from gridmap_msg import GridMap

def callback_todo(data_perception):
	pass

def listener():
	rospy.init_node("surrounding_build", anonymous=True)
	# subscribers

	rospy.Subscriber("todo", type_todo, callback_todo, queue_size=1)

	# publishers
	pub_grid_map = rospy.Publisher("/grid_map", GridMap, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



