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
# ros cycle(s)
r_c = 0.1
# time to take effect
TTE = 0.3
tte = TTE
coe_tte = 0.5

def callback_active_task(active_task_code):
	global task_code
	task_code = active_task_code.data


def callback_fuzzy_curv_info(curv_type):
	global fuzzy_curv_type
	fuzzy_curv_type = curv_type.data


def callback_deviation_from_lane_center(deviation):
	global deviation_from_lane_center, dist_to_left_lane_line, dist_to_right_lane_line
	dist_to_left_lane_line, dist_to_right_lane_line = deviation.data[0], deviation.data[1]
	deviation_from_lane_center = (dist_to_left_lane_line - dist_to_right_lane_line)/2


def callback_ego_speed(speed):
	global ego_speed
	ego_speed = speed.data


def callback_distance_to_object_on_my_path(distance):
	global distance_to_object_on_my_path
	distance_to_object_on_my_path = distance.data


def callback_speed_of_object_on_my_path(speed):
	global speed_of_object_on_my_path
	speed_of_object_on_my_path = speed.data


def listener():
	rospy.init_node("maneuver", anonymous=True)
	# subscribers

	rospy.Subscriber("/active_task_code", Int8, callback_active_task, queue_size=1)
	rospy.Subscriber("/fuzzy_curv_info", Int8, callback_fuzzy_curv_info, queue_size=1)
	rospy.Subscriber("/deviation_from_lane_center", Float32, callback_deviation_from_lane_center, queue_size=1)
	rospy.Subscriber("/ego_speed", Float32, callback_ego_speed, queue_size=1)
	rospy.Subscriber("/distance_to_object_on_my_path", Float32, callback_distance_to_object_on_my_path, queue_size=1)
	rospy.Subscriber("/speed_of_object_on_my_path", Float32, callback_speed_of_object_on_my_path, queue_size=1)

	# publishers
	pub_steering = rospy.Publisher("/control/steering_angle", Float32, queue_size=1)
    pub_steering_speed_limit = rospy.Publisher("/control/steer_speed_limit", Float32, queue_size=1)
	pub_torque = rospy.Publisher("/torque", Float32, queue_size=1)
	pub_brake = rospy.Publisher("/brake", Float32, queue_size=1)
	
	# 10 times per second
	rate = rospy.Rate(1/r_c)

	global task_code
	task_code = 0

	global fuzzy_curv_type
	fuzzy_curv_type = 4
	last_fuzzy_curv_type = 4

	global dist_to_left_lane_line, dist_to_right_lane_line
	dist_to_left_lane_line = 0.0
	dist_to_right_lane_line = 0.0

	global deviation_from_lane_center
	deviation_from_lane_center = 0.0
	last_deviation_from_lane_center = 0.0
	
	global ego_speed
	ego_speed = 0.0

	global distance_to_object_on_my_path
	distance_to_object_on_my_path = None

	global speed_of_object_on_my_path
	speed_of_object_on_my_path = None

	# controls
	steering_angle = 0.0
	steering_speed_limit = 100.0
	torque = 0.0
	brake = 0.0

	# fuzzy
	fs = FuzzySteer()
	# initialize
	fuzzy_curv = fs.curv_fuzzy_set[fuzzy_curv_type] # 'CURV_ZO'
	steering_control_step = fs.getSteeringAngleControlStep(fuzzy_curv)

	# TTE count
	tte_count = 0

	while not rospy.is_shutdown():
		if task_code == 0:
			# deviation rate
			deviation_rate = np.abs((deviation_from_lane_center - last_deviation_from_lane_center) / r_c)
			tte = (coe_tte * dist_to_left_lane_line / deviation_rate) if dist_to_left_lane_line < dist_to_right_lane_line else (coe_tte * dist_to_right_lane_line / deviation_rate)
			# less than 1 sec, greater than r_c
			tte = 1.0 if tte >= 1.0 else tte
			tte = r_c if tte <= r_c else tte
			
			# defuzzification
			fuzzy_curv = fs.curv_fuzzy_set(fuzzy_curv_type)
			steering_control_step = fs.getSteeringAngleControlStep(fuzzy_curv)
			# new control
			if fuzzy_curv_type != last_fuzzy_curv_type:
				tte_count = 0 # for time to take effect
				defuzzied_steering_angle = fs.getDefuzzificationByFuzzy(fuzzy_curv)
				# lcc & acc
				steering_angle, torque, brake = lane_center_and_adaptive_cruise_control(defuzzied_steering_angle,
 																						deviation_from_lane_center,
																						last_deviation_from_lane_center,
 																						ego_speed,
																						steering_control_step,
 																						insensitiveness,
																						distance_to_object_on_my_path,
																						speed_of_object_on_my_path)
			else:
				# after controls deployed for tte seconds, we consider deploying new controls, otherwise use old controls
				if tte_count % (tte/r_c) == 0:
					tte_count = 0
					# lcc & acc
					steering_angle, torque, brake = lane_center_and_adaptive_cruise_control(steering_angle,
 																							deviation_from_lane_center,
																							last_deviation_from_lane_center,
 																							ego_speed,
																							steering_control_step,
 																							insensitiveness,
																							distance_to_object_on_my_path,
																							speed_of_object_on_my_path)
				else:
					# use old controls
					pass

			# tte_count count up
			tte_count++
			# update last_fuzzy_curv
			last_fuzzy_curv_type = fuzzy_curv_type
			# update last_deviation_from_lane_center
			last_deviation_from_lane_center = deviation_from_lane_center

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
	



