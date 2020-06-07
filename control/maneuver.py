#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os
from fuzzy_control_reference import *

# deviation insensitiveness
insensitiveness = 0.1
# ros cycle
r_c = 0.1
# time to take effect
TTE = 0.3

def callback_active_task(active_task_code):
	global task_code
	task_code = active_task_code.data


def callback_fuzzy_curv(data_fuzzy_curv):
	global fuzzy_curv
	fuzzy_curv = data_fuzzy_curv.data


def callback_deviation(data_deviation):
	global deviation
	deviation = data_deviation.data


def callback_ego_speed(data_ego_speed):
	global ego_speed
	ego_speed = data_ego_speed.data


def callback_distance_to_front_vehicle_or_pedestrian(data_distance_to_front_vehicle_or_pedestrian):
	global distance_to_front_vehicle_or_pedestrian
	distance_to_front_vehicle_or_pedestrian = data_distance_to_front_vehicle_or_pedestrian.data


def callback_speed_of_front_vehicle_or_pedestrian(data_speed_of_front_vehicle_or_pedestrian):
	global speed_of_front_vehicle_or_pedestrian
	speed_of_front_vehicle_or_pedestrian = data_speed_of_front_vehicle_or_pedestrian.data


def listener():
	rospy.init_node("maneuver", anonymous=True)
	# subscribers

	rospy.Subscriber("/active_task_code", Int8, callback_active_task, queue_size=1)
	rospy.Subscriber("/active_task_code", Int8, callback_active_task, queue_size=1)

	# publishers
	pub_steering = rospy.Publisher("/control/steering_angle", Float32, queue_size=1)
    pub_steering_speed_limit = rospy.Publisher("/control/steer_speed_limit", Float32, queue_size=1)
	pub_torque = rospy.Publisher("/torque", Float32, queue_size=1)
	pub_brake = rospy.Publisher("/brake", Float32, queue_size=1)
	
	# 10 times per second
	rate = rospy.Rate(1/r_c)

	global task_code
	task_code = 0

	global fuzzy_curv
	fuzzy_curv = 0
	last_fuzzy_curv = 0

	global deviation
	deviation = 0.0
	
	global ego_speed
	ego_speed = 0.0

	global distance_to_front_vehicle_or_pedestrian
	distance_to_front_vehicle_or_pedestrian = None

	global speed_of_front_vehicle_or_pedestrian
	speed_of_front_vehicle_or_pedestrian = None

	# controls
	steering_angle = 0.0
	steering_speed_limit = 100.0
	torque = 0.0
	brake = 0.0

	# fuzzy
	fuzzy_steer_control = FuzzySteer()
	in_curv_fuzzy = fuzzy_steer_control.curv_fuzzy_set[4] # 'CURV_ZO'
	steering_step = fuzzy_steer_control.getSteeringAngleControlStep(in_curv_fuzzy)

	# TTE count
	tte_count = 0

	while not rospy.is_shutdown():
		if task_code == 0:
			in_curv_fuzzy = fuzzy_steer_control.curv_fuzzy_set(fuzzy_curv)
			steering_step = fuzzy_steer_control.getSteeringAngleControlStep(in_curv_fuzzy)
			if fuzzy_curv != last_fuzzy_curv:
				tte_count = 0
				defuzzied_steering_angle = fuzzy_steer_control.getDefuzzificationByFuzzy(in_curv_fuzzy)
				# lcc & acc
				steering_angle, torque, brake = lane_center_and_adaptive_cruise_control(defuzzied_steering_angle,
 																						deviation,
 																						ego_speed,
																						steering_step,
 																						insensitiveness,
																						distance_to_front_vehicle_or_pedestrian,
																						speed_of_front_vehicle_or_pedestrian)
			else:
				if tte_count % (TTE/r_c) == 0:
					tte_count = 0
					# lcc & acc
					steering_angle, torque, brake = lane_center_and_adaptive_cruise_control(steering_angle,
 																						deviation,
 																						ego_speed,
																						steering_step,
 																						insensitiveness,
																						distance_to_front_vehicle_or_pedestrian,
																						speed_of_front_vehicle_or_pedestrian)

			# tte_count count up
			tte_count++
			# update last_fuzzy_curv
			last_fuzzy_curv = fuzzy_curv

		elif task_code == 1:
			#steering_angle, torque, brake = lane_change_control()
			pass
		
		elif task_code == 2:
			#steering_angle, torque, brake = traveling_across_intersection()
			pass
				
		
		# publish controls
		pub_steering.publish(Float32(steering_angle))
		pub_steering_speed_limit.publish(Float32(steering_speed_limit))
		pub_torque.publish(Float32(torque))
		rate.sleep(Float32(brake))

	rospy.spin()


if __name__ == '__main__':
	listener()
	



