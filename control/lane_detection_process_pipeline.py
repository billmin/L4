import os, sys
import numpy as np
import json
import matplotlib.pyplot as plt
import scipy.interpolate, scipy.optimize
from pixpos2realpos import estimate_real_location


class LaneDetectionProcessPipeline:
	CAM_WIDTH = 640
	CAM_HEIGHT = 360
	NUGET_SIZE = 0.12

	def _polyfit_solver(x, y, n):
		f = np.polyfit(x, y, n)
		p = np.poly1d(f)
	
		return p

	def _intercept_solver(x, y1, y2):
		assert len(x)==len(y1)==len(y2), "dimension conflict!"

		interp1 = scipy.interpolate.InterpolatedUnivariateSpline(x, y1)
		interp2 = scipy.interpolate.InterpolatedUnivariateSpline(x, y2)

		new_x = np.linspace(x.min(), x.max(), 100)
		new_y1 = interp1(new_x)
		new_y2 = interp2(new_x)

		def __difference(x):
			return np.abs(interp1(x) - interp2(x))

		x_at_crossing = scipy.optimize.fsolve(__difference, x0=3.0)

		return x_at_crossing, interp1, interp2


	def __init__(self, 
				 left_front_pix_real_pos_map,
				 right_front_pix_real_pos_map,
				 left_rear_pix_real_pos_map,
				 right_rear_pix_real_pos_map,
				 left_front_ref,
				 right_front_ref,
				 left_rear_ref,
				 right_rear_ref,
				 left_front_lane_line_detection,
				 right_front_lane_line_detection,
				 left_rear_lane_line_detection,
				 right_rear_lane_line_detection,
				 front_view_check,
				 left_side_view_check,
				 right_side_view_check):
		'''
		left_front_pix_real_pos_map:	mapping of pixel pos to real pos of left front view
		right_front_pix_real_pos_map:	mapping of pixel pos to real pos of right front view
		left_rear_pix_real_pos_map:		mapping of pixel pos to real pos of left rear view
		right_rear_pix_real_pos_map:	mapping of pixel pos to real pos of right rear view
		left_front_ref:					reference line of left front wheel, which is contained in dict
		right_front_ref:				reference line of right front wheel, which is contained in dict
		left_rear_ref:					reference line of left rear wheel, which is contained in dict
		right_rear_ref:					reference line of right rear wheel, which is contained in dict
		left_front_lane_line_detection:	lane line detection of left front deadzone cam (list of dict)
		right_front_lane_line_detection:lane line detection of right front deadzone cam
		left_rear_lane_line_detection:	lane line detection of left rear deadzone cam
		right_rear_lane_line_detection:	lane line detection of right rear deadzone cam
		front_view_check:				check situation in front
		left_side_view_check:			check situation on left side
		right_side_view_check:			check situation on right side
		'''
		# pixel map to real
		self._left_front_pix_real_pos_map = left_front_pix_real_pos_map
		self._right_front_pix_real_pos_map = right_front_pix_real_pos_map
		self._left_rear_pix_real_pos_map = left_rear_pix_real_pos_map
		self._right_rear_pix_real_pos_map = right_rear_pix_real_pos_map
		# polyfit functions of ref lines
		self._left_front_ref_func = _polyfit_solver(np.array(left_front_ref['x']), np.array(left_front_ref['y']), 2)
		self._right_front_ref_func = _polyfit_solver(np.array(right_front_ref['x']), np.array(right_front_ref['y']), 2)
		self._left_rear_ref_func = _polyfit_solver(np.array(left_rear_ref['x']), np.array(left_rear_ref['y']), 2)
		self._right_rear_ref_func = _polyfit_solver(np.array(right_rear_ref['x']), np.array(right_rear_ref['y']), 2)
		#########################################
		## lane detection information		
		## SingleLineStamped.msg:
		##		int8 		id
		##		int8 		point_num
		##		float32 	score
		##		float32[72] positionx
		##		float32[72] positiony
		##		int8		xushi
		## AllLinesStamped.msg:
		##		std_msgs/Header		header
		##		uint8				line_number
		##		SingleLineStamped[] all_lines
		#########################################
		# lane line detection (AllLinesStamped type)
		self._left_front_lane_line_detection = left_front_lane_line_detection
		self._right_front_lane_line_detection = right_front_lane_line_detection
		self._left_rear_lane_line_detection = left_rear_lane_line_detection
		self._right_rear_lane_line_detection = right_rear_lane_line_detection

		# check situation around
		self._front_view_check = front_view_check
		self._left_side_view_check = left_side_view_check
		self._right_side_view_check = right_side_view_check

		# ego lane line (SingleLineStamped type)
		self._left_front_ego_lane_line = {}
		self._right_front_ego_lane_line = {}
		self._left_rear_ego_lane_line = {}
		self._right_rear_ego_lane_line = {}

		# neighbor lane line (SingleLineStamped type)
		self._left_front_neighbor_lane_line = {}
		self._right_front_neighbor_lane_line = {}
		self._left_rear_neighbor_lane_line = {}
		self._right_rear_neighbor_lane_line = {}

		# intercept with ego lane line
		self._left_front_intercept_with_ego_lane_line = np.array([0.0, 0.0])
		self._right_front_intercept_with_ego_lane_line = np.array([0.0, 0.0])
		self._left_rear_intercept_with_ego_lane_line = np.array([0.0, 0.0])
		self._right_rear_intercept_with_ego_lane_line = np.array([0.0, 0.0])

		# intercept with neighbor lane line
		self._left_front_intercept_with_neighbor_lane_line = np.array([0.0, 0.0])
		self._right_front_intercept_with_neighbor_lane_line = np.array([0.0, 0.0])
		self._left_rear_intercept_with_neighbor_lane_line = np.array([0.0, 0.0])
		self._right_rear_intercept_with_neighbor_lane_line = np.array([0.0, 0.0])
		# distance to ego lane line
		self._distance_from_head_to_left_front_ego_lane_line = 0.0
		self._distance_from_head_to_right_front_ego_lane_line = 0.0
		self._distance_from_wheel_to_left_rear_ego_lane_line = 0.0
		self._distance_from_wheel_to_right_rear_ego_lane_line = 0.0


	@property
	def distance_from_head_to_left_front_ego_lane_line(self):
		return self._distance_from_head_to_left_front_ego_lane_line

	@property
	def distance_from_head_to_right_front_ego_lane_line(self):
		return self._distance_from_head_to_right_front_ego_lane_line

	@property
	def distance_from_wheel_to_left_rear_ego_lane_line(self):
		return self._distance_from_wheel_to_left_rear_ego_lane_line
	
	@property
	def distance_from_wheel_to_right_rear_ego_lane_line(self):
		return self._distance_from_wheel_to_right_rear_ego_lane_line


	def __call__(self, 
				 left_front_lane_line_detection,
				 right_front_lane_line_detection,
				 left_rear_lane_line_detection,
				 right_rear_lane_line_detection,
				 front_view_check,
				 left_side_view_check,
				 right_side_view_check):
		# update lane line detection
		self._left_front_lane_line_detection = left_front_lane_line_detection
		self._right_front_lane_line_detection = right_front_lane_line_detection
		self._left_rear_lane_line_detection = left_rear_lane_line_detection
		self._right_rear_lane_line_detection = right_rear_lane_line_detection
		# update check
		self._front_view_check = front_view_check
		self._left_side_view_check = left_side_view_check
		self._right_side_view_check = right_side_view_check
		

	def update_ego_lane_line(self):
		# do some filtering thing

	def update_intercept_with_ego_lane_line(self):
		##############
		# left front
		##############
		pos_x_lf = self._left_front_ego_lane_line['positionx']
		pos_y_lf = self._left_front_ego_lane_line['positiony']
		# delete zero
		del_list = []

		# non-zero data points
		data_len = pos_x_lf.shape[0]
		for i in range(data_len):
			if pos_x_lf[i]==pos_y_lf[i]==0:
				del_list.append(i)
		pos_x_lf = np.delete(pos_x_lf, del_list)
		pos_y_lf = np.delete(pos_y_lf, del_list)

		# check order
		if pos_x_lf[0] > pos_x_lf[-1]:
			pos_x_lf = pos_x_lf[::-1]
			pos_y_lf = pos_y_lf[::-1]

		# x, y1, y2
		if data_len < 4:
			# select domain of x, evenly distributed, it takes longer time!
			x_domain_lf = np.linspace(pos_x_lf[0], pos_x_lf[-1], 10)
			# calc y values of ref line corresponding to domain of x
			y_ref_lf = self._left_front_ref_func(x_domain_lf)
			# fit pos_y_lf
			pos_y_lf_func = _polyfit_solver(pos_x_lf, pos_y_lf, 2)
			# calc y values of left front ego lane line corresponding to domain of x
			new_pos_y_lf = pos_y_lf_func(x_domain_lf)
			# calc intercept
			x_cross_lf, _, _ = _intercept_solver(x_domain_lf, y_ref_lf, new_pos_y_lf)
			y_cross_lf = self._left_front_ref_func(x_cross_lf)
			# update intercept
			self._left_front_intercept_with_ego_lane_line = np.array([x_cross_lf, y_cross_lf])
		else:
			# it is reasonable to take pos_x_lf as evenly distributed
			x_domain_lf = pos_x_lf
			# calc y values of ref line corresponding to domain of x
			y_ref_lf = self._left_front_ref_func(x_domain_lf)
			# calc y values of left front ego lane line corresponding to domain of x
			new_pos_y_lf = pos_y_lf
			# calc intercept
			x_cross_lf, _, _ = _intercept_solver(x_domain_lf, y_ref_lf, new_pos_y_lf)
			y_cross_lf = self._left_front_ref_func(x_cross_lf) 
			# update intercept
			self._left_front_intercept_with_ego_lane_line = np.array([x_cross_lf, y_cross_lf])
		
		##############
		# right front
		##############
		pos_x_rf = self._right_front_ego_lane_line['positionx']
		pos_y_rf = self._right_front_ego_lane_line['positiony']
		# delete zero
		del_list = []

		# non-zero data points
		data_len = pos_x_rf.shape[0]
		for i in range(data_len):
			if pos_x_rf[i]==pos_y_rf[i]==0:
				del_list.append(i)
		pos_x_rf = np.delete(pos_x_rf, del_list)
		pos_y_rf = np.delete(pos_y_rf, del_list)

		# check order
		if pos_x_rf[0] > pos_x_rf[-1]:
			pos_x_rf = pos_x_rf[::-1]
			pos_y_rf = pos_y_rf[::-1]

		# x, y1, y2
		if data_len < 4:
			# select domain of x, evenly distributed, it takes longer time!
			x_domain_rf = np.linspace(pos_x_rf[0], pos_x_rf[-1], 10)
			# calc y values of ref line corresponding to domain of x
			y_ref_rf = self._right_front_ref_func(x_domain_rf)
			# fit pos_y_rf
			pos_y_rf_func = _polyfit_solver(pos_x_rf, pos_y_rf, 2)
			# calc y values of left front ego lane line corresponding to domain of x
			new_pos_y_rf = pos_y_rf_func(x_domain_rf)
			# calc intercept
			x_cross_rf, _, _ = _intercept_solver(x_domain_rf, y_ref_rf, new_pos_y_rf)
			y_cross_rf = self._right_front_ref_func(x_cross_rf)
			# update intercept
			self._right_front_intercept_with_ego_lane_line = np.array([x_cross_rf, y_cross_rf])
		else:
			# it is reasonable to take pos_x_lf as evenly distributed
			x_domain_rf = pos_x_rf
			# calc y values of ref line corresponding to domain of x
			y_ref_rf = self._right_front_ref_func(x_domain_rf)
			# calc y values of left front ego lane line corresponding to domain of x
			new_pos_y_rf = pos_y_rf
			# calc intercept
			x_cross_rf, _, _ = _intercept_solver(x_domain_rf, y_ref_rf, new_pos_y_rf)
			y_cross_rf = self._right_front_ref_func(x_cross_rf)
			# update intercept
			self._right_front_intercept_with_ego_lane_line = np.array([x_cross_rf, y_cross_rf])
		
		#############
		# left rear 
		#############
		pos_x_lr = self._left_rear_ego_lane_line['positionx']
		pos_y_lr = self._left_rear_ego_lane_line['positiony']
		# delete zero
		del_list = []

		# non-zero data points
		data_len = pos_x_lr.shape[0]
		for i in range(data_len):
			if pos_x_lr[i]==pos_y_lr[i]==0:
				del_list.append(i)
		pos_x_lr = np.delete(pos_x_lr, del_list)
		pos_y_lr = np.delete(pos_y_lr, del_list)

		# check order
		if pos_x_lr[0] > pos_x_lr[-1]:
			pos_x_lr = pos_x_lr[::-1]
			pos_y_lr = pos_y_lr[::-1]

		# x, y1, y2
		if data_len < 4:
			# select domain of x, evenly distributed, it takes longer time!
			x_domain_lr = np.linspace(pos_x_lr[0], pos_x_lr[-1], 10)
			# calc y values of ref line corresponding to domain of x
			y_ref_lr = self._left_rear_ref_func(x_domain_lr)
			# fit pos_y_lr
			pos_y_lr_func = _polyfit_solver(pos_x_lr, pos_y_lr, 2)
			# calc y values of left front ego lane line corresponding to domain of x
			new_pos_y_lr = pos_y_lr_func(x_domain_lr)
			# calc intercept
			x_cross_lr, _, _ = _intercept_solver(x_domain_lr, y_ref_lr, new_pos_y_lr)
			y_cross_lr = self._left_rear_ref_func(x_cross_lr)
			# update intercept
			self._left_rear_intercept_with_ego_lane_line = np.array([x_cross_lr, y_cross_lr])
		else:
			# it is reasonable to take pos_x_lf as evenly distributed
			x_domain_lr = pos_x_lr
			# calc y values of ref line corresponding to domain of x
			y_ref_lr = self._left_rear_ref_func(x_domain_lr)
			# calc y values of left front ego lane line corresponding to domain of x
			new_pos_y_lr = pos_y_lr
			# calc intercept
			x_cross_lr, _, _ = _intercept_solver(x_domain_lr, y_ref_lr, new_pos_y_lr)
			y_cross_lr = self._left_rear_ref_func(x_cross_lr) 
			# update intercept
			self._left_rear_intercept_with_ego_lane_line = np.array([x_cross_lr, y_cross_lr])

		#############
		# right rear
		#############
		pos_x_rr = self._right_rear_ego_lane_line['positionx']
		pos_y_rr = self._right_rear_ego_lane_line['positiony']
		# delete zero
		del_list = []

		# non-zero data points
		data_len = pos_x_rr.shape[0]
		for i in range(data_len):
			if pos_x_rr[i]==pos_y_rr[i]==0:
				del_list.append(i)
		pos_x_rr = np.delete(pos_x_rr, del_list)
		pos_y_rr = np.delete(pos_y_rr, del_list)

		# check order
		if pos_x_rr[0] > pos_x_rr[-1]:
			pos_x_rr = pos_x_rr[::-1]
			pos_y_rr = pos_y_rr[::-1]

		# x, y1, y2
		if data_len < 4:
			# select domain of x, evenly distributed, it takes longer time!
			x_domain_rr = np.linspace(pos_x_rr[0], pos_x_rr[-1], 10)
			# calc y values of ref line corresponding to domain of x
			y_ref_rr = self._right_rear_ref_func(x_domain_rr)
			# fit pos_y_rr
			pos_y_rr_func = _polyfit_solver(pos_x_rr, pos_y_rr, 2)
			# calc y values of left front ego lane line corresponding to domain of x
			new_pos_y_rr = pos_y_rr_func(x_domain_rr)
			# calc intercept
			x_cross_rr, _, _ = _intercept_solver(x_domain_rr, y_ref_rr, new_pos_y_rr)
			y_cross_rr = self._right_rear_ref_func(x_cross_rr)
			# update intercept
			self._right_rear_intercept_with_ego_lane_line = np.array([x_cross_rr, y_cross_rr])
		else:
			# it is reasonable to take pos_x_lf as evenly distributed
			x_domain_rr = pos_x_rr
			# calc y values of ref line corresponding to domain of x
			y_ref_rr = self._right_rear_ref_func(x_domain_rr)
			# calc y values of left front ego lane line corresponding to domain of x
			new_pos_y_rr = pos_y_rr
			# calc intercept
			x_cross_rr, _, _ = _intercept_solver(x_domain_rr, y_ref_rr, new_pos_y_rr)
			y_cross_rr = self._right_rear_ref_func(x_cross_rr)
			# update intercept
			self._right_rear_intercept_with_ego_lane_line = np.array([x_cross_rr, y_cross_rr])


	def update_neighbor_lane_line(self):
		# do some filtering thing
		pass

	def update_intercept_with_neighbor_lane_line(self):
		pass

	def update_distances_from_wheels_to_ego_lane_lines(self):
		# left front
		pix_x, pix_y = self._left_front_intercept_with_ego_lane_line
		pix_x /= CAM_WIDTH
		pix_y /= CAM_HEIGHT
		self._distance_from_head_to_left_front_ego_lane_line = estimate_real_location(pix_x, pix_y, self._left_front_pix_real_pos_map)[0]

		# right front
		pix_x, pix_y = self._right_front_intercept_with_ego_lane_line
		pix_x /= CAM_WIDTH
		pix_y /= CAM_HEIGHT
		self._distance_from_head_to_right_front_ego_lane_line = estimate_real_location(pix_x, pix_y, self._right_front_pix_real_pos_map)[0]

		# left rear
		pix_x, pix_y = self._left_rear_intercept_with_ego_lane_line
		pix_x /= CAM_WIDTH
		pix_y /= CAM_HEIGHT
		self._distance_from_wheel_to_left_rear_ego_lane_line = estimate_real_location(pix_x, pix_y, self._left_rear_pix_real_pos_map)[0]
		
		# right rear
		pix_x, pix_y = self._right_rear_intercept_with_ego_lane_line
		pix_x /= CAM_WIDTH
		pix_y /= CAM_HEIGHT
		self._distance_from_wheel_to_right_rear_ego_lane_line = estimate_real_location(pix_x, pix_y, self._right_rear_pix_real_pos_map)[0]



if __name__ == '__main__':

	with open('coords_x_axis_rr.json', 'r') as file:
		# get all pxiel2real coordinate pairs
		coords_x_axis = json.load(file)

		for k, v in coords_x_axis.items():
			print(k, v)
		
