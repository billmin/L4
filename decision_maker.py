#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
# import pyqtgraph as pg
import time
import copy
import os

# recieved messages
#from guider_msgs.msg import SidePoints

# published messages
#from planner_msgs.msg import Trajectory, InfoFrenetPlanner

def callback_navigation(navi_info):
	global lidar_perceptions, lidar_perceptions_received
	lidar_perceptions = data_lidar_perception
	lidar_perceptions_received = True
	# print(lidar_perceptions.perceptions[0])

def listner():
	rospy.init_node("decision_maker", anonymous=True)
	# subscribers
	rospy.Subscriber("/obstacle", SidePoints, callback_obstacle, queue_size=1)
	rospy.Subscriber("/gps_info", Gprmc, callback_gps, queue_size=1)
	rospy.Subscriber("/imu_info", Imu, callback_imu)
	rospy.Subscriber("/path_points", PlanToControl, callback_waypoints, queue_size=1)
	rospy.Subscriber("/stopline_detect", Float32MultiArray, callback_stopline_detect, queue_size=1)
	rospy.Subscriber("/center_line", Float32MultiArray, callback_center_line, queue_size=1)
	rospy.Subscriber("/rs_percept_result", PerceptionListMsg, callback_lidar_perception, queue_size=1)

	# publishers
	pub_perception = rospy.Publisher('planner_obs_dynamic', PerceptionMsg, queue_size=1)

	rate = rospy.Rate(10)

	global obstacle
	global received_new_obstacle
	global unprocessed_obstacle
	obstacle = SidePoints().side
	received_new_obstacle = False

	#global center_line
	#center_line = Float32MultiArray().data
	#center_line_pre = copy.copy(center_line)

	global lidar_perceptions, lidar_perceptions_received
	lidar_perceptions = PerceptionListMsg()
	lidar_perceptions_received = False


	while not rospy.is_shutdown():
	
		# time control
		loop_time_end = time.time()
		time_total = loop_time_end - t_start_loop

		rate.sleep()
	rospy.spin()


if __name__ == '__main__':
	listner()
	



