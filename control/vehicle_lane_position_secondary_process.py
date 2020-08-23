import os, sys
import numpy as np
import time


def check_validity_of_all_distances_to_ego_lane_lines(c_speed,
													  c_dev_angle,
													  distance_from_head_to_left_front_ego_lane_line,
													  distance_from_head_to_right_front_ego_lane_line,
													  distance_from_wheel_to_left_rear_ego_lane_line,
													  distance_from_wheel_to_right_rear_ego_lane_line,
													  historical_distance_left_front,
													  historical_distance_right_front,
													  historical_distance_left_rear,
													  historical_distance_right_rear):

	def _next_data_approximation(historical_data, current_dev_angle, current_speed):
		if len(historical_data) >= 2:
			a = historical_data[-2]
			b = historical_data[-1]
			# approximate
			approximate_0 = 2*b-a
			approximate_1 = b + current_speed*t*np.tan(current_dev_angle*np.pi/180.0)
			# expected
			return 0.5*(approximate_0 + approximate_1)
		elif len(historical_data) == 1:
			a = historical_data[0]
			# approximate
			approximate = a + current_speed*t*np.tan(current_dev_angle*np.pi/180.0)
			# expected
			return approximate
		else:
			return None
	

	def _outlier_assessment(approximated, measured):
		epsilon = 0.00000001
		threshold = 0.25

		if approximated is None:
			return False, measured
		else:
			if np.abs(approximated - measured)/(approximated+epsilon) <= threshold:
				return False, measured
			else:
				return True, None

	## approximate all distances
	next_approximate_distance_left_front = _next_data_approximation(historical_distance_left_front)
	next_approximate_distance_right_front = _next_data_approximation(historical_distance_right_front)
	next_approximate_distance_left_rear = _next_data_approximation(historical_distance_left_rear)
	next_approximate_distance_right_rear = _next_data_approximation(historical_distance_right_rear)

	## lf
	# filtering left front
	is_lf_outlier, assessed_lf = _outlier_assessment(next_approximate_distance_left_front, distance_from_head_to_left_front_ego_lane_line)

	## rf
	# filtering right front
	is_rf_outlier, assessed_rf = _outlier_assessment(next_approximate_distance_right_front, distance_from_head_to_right_front_ego_lane_line)

	## lr
	# filtering left rear
	is_lr_outlier, assessed_lr = _outlier_assessment(next_approximate_distance_left_rear, distance_from_wheel_to_left_rear_ego_lane_line)

	## rr
	# filtering right rear
	is_rr_outlier, assessed_rr = _outlier_assessment(next_approximate_distance_right_rear, distance_from_wheel_to_right_rear_ego_lane_line)

	return is_lf_outlier, is_rf_outlier, is_lr_outlier, is_rr_outlier, assessed_lf, assessed_rf, assessed_lr, assessed_rr

