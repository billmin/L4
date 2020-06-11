import os, sys


def efficient_coarse_tuning(current_deviation_state, last_deviation_state, control_step):
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
		delta_control = pre_delta_control if p > 0 else 0.5 * pre_delta_control	
			
	# update last deviation state
	last_deviation_state = current_deviation_state

	return delta_control


def lane_center_and_adaptive_cruise_control(current_steering_angle,
											deviation_from_lane_center,
											last_deviation_from_lane_center,
											ego_speed,
											steering_control_step
											insensitiveness=0.1,
											distance_to_front_vehicle_or_pedestrian=None,
											speed_of_front_vehicle_or_pedestrian=None):
	
	# lcc
	steer_direct_setup_coe = -1 # (when steering to left, steering angle is positive on A3G9D5)

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

	# incremental steering angle
	delta_steering_angle = efficient_coarse_tuning(current_deviation_state, last_deviation_state, steering_control_step)	
	steering_angle = current_steering_angle + steer_direct_setup_coe * delta_steering_angle

	# acc
	torque = 150.0
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
