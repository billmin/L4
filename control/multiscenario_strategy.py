import os, sys
import numpy as np

CAR_TYPE = '3g9d5' # or '3g9d5'

# turn radius corresponding to four wheels: left front, right front, left rear, right rear
def turn_radius_left_front_wheel(wheel_base, angle_left_front_wheel):
	return wheel_base / np.sin(angle_left_front_wheel)

def turn_radius_right_front_wheel(wheel_base, angle_right_front_wheel):
	return wheel_base / np.sin(angle_right_front_wheel)

def turn_radius_left_rear_wheel(wheel_base, angle_left_rear_wheel):
	return wheel_base / np.tan(angle_left_rear_wheel)

def turn_radius_right_rear_wheel(wheel_base, angle_right_rear_wheel):
	return wheel_base / np.tan(angle_right_rear_wheel)

# approximation
def turn_radius_ceter_vehicle(turn_radius_left_front, turn_radius_right_front):
	return 0.5 * (turn_radius_left_front + turn_radius_right_front)

def steering_angle_to_wheel_angle(steering_angle):
	wheel_angle = -0.0607 * steering_angle + 0.0082
	# limitation
	if wheel_angle > 34.18176116:
		wheel_angle = 34.18176116
	elif wheel_angle < -34.17822892:
		wheel_angle = -34.17822892

	return wheel_angle

def wheel_angle_to_steering_angle(wheel_angle):
	steering_angle = -16.407 * wheel_angle + 0.1357
	# limits
	if steering_angle > 519.0:
		steering_angle = 519.0
	elif steering_angle < -519.0:
		steering_angle = -519.0

	return steering_angle

def limited_turn_radius(c_speed, limited_lateral_acceleration):
	min_radius = 3.4 # minimum turn radius
	limited_radius = c_speed * c_speed / limited_lateral_acceleration
	if limited_radius < min_radius:
		limited_radius = min_radius

	return min_radius

def limited_ego_speed(expected_turn_radius, limited_lateral_acceleration):
	return np.sqrt(expected_turn_radius*limited_lateral_acceleration)


def efficient_coarse_tuning(current_deviation_state, last_deviation_state, diff_deviation_state, control_step):
	'''
	current_deviation_state:		current deviation state -1: deviation to left, 0: deviation ignorable, +1: deviation to right
	last_deviation_state: 			deviation state of last moment
	control_step: 					step of controls
	'''
	if last_deviation_state == 0: # turning point when last_deviation_state is 0
		delta_control = (-current_deviation_state) * control_step
	else:
		p = current_deviation_state * last_deviation_state
		pre_delta_control = (-current_deviation_state) * control_step
		if p > 0:
			if diff_deviation_state == 1:
				delta_control = pre_delta_control
			elif diff_deviation_state == -1:
				delta_control = 0.0
			else:
				delta_control = 0.5*pre_delta_control
		elif p == 0:
			delta_control = 0.0
		else:
			if diff_deviation_state == 1:
				delta_control = 1.5*pre_delta_control
			elif diff_deviation_state == -1:
				delta_control = 0.0
			else:
				delta_control = pre_delta_control

	return delta_control


def lane_center_and_adaptive_cruise_control(current_steering_angle,
											deviation_from_lane_center,
											last_deviation_from_lane_center,
											ego_speed,
											steering_control_step,
											insensitiveness=0.02,
											distance_to_front_vehicle_or_pedestrian=None,
											speed_of_front_vehicle_or_pedestrian=None):
	
	# lcc
	if CAR_TYPE == '3g9d5':
		steer_direct_setup_coe = -1 # (when steering to left, steering angle is positive on A3G9D5)
	else:
		steer_direct_setup_coe = 1

	# check last deviation state
	if -insensitiveness <= last_deviation_from_lane_center <= insensitiveness:
		last_deviation_state = 0
	elif last_deviation_from_lane_center < -insensitiveness:
		last_deviation_state = -1
	else:
		last_deviation_state = 1

	# check current deviation state
	if -insensitiveness <= deviation_from_lane_center <= insensitiveness:
		current_deviation_state = 0
	elif deviation_from_lane_center < -insensitiveness:
		current_deviation_state = -1
	else:
		current_deviation_state = 1

	# diff deviation
	diff_deviation = abs(deviation_from_lane_center) - abs(last_deviation_from_lane_center)
	if diff_deviation < 0:
		diff_deviation_state = -1
	elif diff_deviation > 0:
		diff_deviation_state = 1
	else:
		diff_deviation_state = 0

	# incremental steering angle
	delta_steering_angle = efficient_coarse_tuning(current_deviation_state, last_deviation_state, diff_deviation_state, steering_control_step)
	#print(delta_steering_angle)	
	steering_angle = current_steering_angle + steer_direct_setup_coe * delta_steering_angle

	# acc
	torque = 0.0
	brake = 0.0

	return steering_angle, torque, brake


def lane_change(lateral_direction_to, 
				stage, 
				ego_speed, 
				distance_to_front_vehicle_in_origin_lane, 
				speed_of_front_vehicle_in_origin_lane,
				distance_to_front_vehicle_in_target_lane,
				speed_of_front_vehicle_in_target_lane):
	pass


def traveling_across_intersection(direction_to, stage, ego_speed):
	pass
