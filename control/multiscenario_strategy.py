import os, sys


def lane_center_and_adaptive_cruise_control(current_steering_angle,
											deviation_from_lane_center,
											ego_speed,
											steering_step
											insensitiveness=0.1,
											distance_to_front_vehicle_or_pedestrian=None,
											speed_of_front_vehicle_or_pedestrian=None):
	
	# lcc (when steering to left, steering angle is positive on A3G9D5)
	if deviation_from_lane_center < -insensitiveness:
		steering_angle = current_steering_angle - steering_step
	elif deviation_from_lane_center > insensitiveness:
		steering_angle = current_steering_angle + steering_step
	else:
		steering_angle = current_steering_angle
	
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
