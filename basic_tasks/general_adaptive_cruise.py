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


def callback_todo(data_todo):
	pass

def callback_active_task(active_task_id):
	global task_id
	task_id = active_task_id.data

def listener():
	rospy.init_node("general_adaptive_cruise", anonymous=True)
	# subscribers
	rospy.Subscriber("/grid_map", GridMap, callback_todo, queue_size=1)
	rospy.Subscriber("/active_task_id", Int8, callback_active_task, queue_size=1)

	# publishers
	pub_steering = rospy.Publisher("/control/steering_angle", Float32, queue_size=1)
    pub_steering_speed_limit = rospy.Publisher("/control/steer_speed_limit", Float32, queue_size=1)
	pub_torque = rospy.Publisher("/torque", Float32, queue_size=1)
	pub_brake = rospy.Publisher("/brake", Float32, queue_size=1)

	rate = rospy.Rate(50)

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



