import os, sys
import numpy as np
import json
import matplotlib.pyplot as plt
import scipy.interpolate, scipy.optimize
from pixpos2realpos import estimate_real_location
from cqpilot_msg.msg import mmt_line
from cqpilot_msg.msg import lines



def polyfit_solver(x, y, n):
	f = np.polyfit(x, y, n)
	p = np.poly1d(f)
	
	return p


def intercept_solver(x, y1, y2, start):
	assert len(x)==len(y1)==len(y2), "dimension conflict!"

	interp1 = scipy.interpolate.InterpolatedUnivariateSpline(x, y1)
	interp2 = scipy.interpolate.InterpolatedUnivariateSpline(x, y2)

	new_x = np.linspace(x.min(), x.max(), 100)
	new_y1 = interp1(new_x)
	new_y2 = interp2(new_x)

	def __difference(x):
		return np.abs(interp1(x) - interp2(x))

	x_at_crossing = scipy.optimize.fsolve(__difference, x0=start)

	return x_at_crossing, interp1, interp2


class LaneDetectionProcessPipeline:
	# image related
	CAM_WIDTH = 640
	CAM_HEIGHT = 360

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
				 front_view_check=None,
				 left_side_view_check=None,
				 right_side_view_check=None):
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
		self._left_front_ref_func = polyfit_solver(np.array(left_front_ref['x']), np.array(left_front_ref['y']), 2)
		self._right_front_ref_func = polyfit_solver(np.array(right_front_ref['x']), np.array(right_front_ref['y']), 2)
		self._left_rear_ref_func = polyfit_solver(np.array(left_rear_ref['x']), np.array(left_rear_ref['y']), 2)
		self._right_rear_ref_func = polyfit_solver(np.array(right_rear_ref['x']), np.array(right_rear_ref['y']), 2)
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
		self._left_front_intercept_with_ego_lane_line = np.array([])
		self._right_front_intercept_with_ego_lane_line = np.array([])
		self._left_rear_intercept_with_ego_lane_line = np.array([])
		self._right_rear_intercept_with_ego_lane_line = np.array([])

		# intercept with neighbor lane line
		self._left_front_intercept_with_neighbor_lane_line = np.array([])
		self._right_front_intercept_with_neighbor_lane_line = np.array([])
		self._left_rear_intercept_with_neighbor_lane_line = np.array([])
		self._right_rear_intercept_with_neighbor_lane_line = np.array([])
		# distance to ego lane line
		self._distance_from_head_to_left_front_ego_lane_line = 0.0
		self._distance_from_head_to_right_front_ego_lane_line = 0.0
		self._distance_from_wheel_to_left_rear_ego_lane_line = 0.0
		self._distance_from_wheel_to_right_rear_ego_lane_line = 0.0
		# historical distance list
		self._historical_distances_from_head_to_left_front_ego_lane_line = []
		self._historical_distances_from_head_to_right_front_ego_lane_line = []
		self._historical_distances_from_wheel_to_left_rear_ego_lane_line = []
		self._historical_distances_from_wheel_to_right_rear_ego_lane_line = []

		# lane detection check
		self._is_left_front_ego_lane_detected = False
		self._is_right_front_ego_lane_detected = False
		self._is_left_rear_ego_lane_detected = False
		self._is_right_rear_ego_lane_detected = False

		# notice: this is not real lane width
		self._lane_width = 3.5


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

	@property
	def is_left_front_ego_lane_detected(self):
		return self._is_left_front_ego_lane_detected

	@property
	def is_right_front_ego_lane_detected(self):
		return self._is_right_front_ego_lane_detected

	@property
	def is_left_rear_ego_lane_detected(self):
		return self._is_left_rear_ego_lane_detected

	@property
	def is_right_rear_ego_lane_detected(self):
		return self._is_right_rear_ego_lane_detected


	def __call__(self, 
				 left_front_lane_line_detection,
				 right_front_lane_line_detection,
				 left_rear_lane_line_detection,
				 right_rear_lane_line_detection,
				 front_view_check=None,
				 left_side_view_check=None,
				 right_side_view_check=None):
		# update lane line detection
		self._left_front_lane_line_detection = left_front_lane_line_detection
		self._right_front_lane_line_detection = right_front_lane_line_detection
		self._left_rear_lane_line_detection = left_rear_lane_line_detection
		self._right_rear_lane_line_detection = right_rear_lane_line_detection
		# update check
		self._front_view_check = front_view_check
		self._left_side_view_check = left_side_view_check
		self._right_side_view_check = right_side_view_check
		

	def update_ego_lane_lines(self):
		# do some filtering thing
		assert isinstance(self._left_front_lane_line_detection, lines), "Wrong type, should be 'lines'!"
		assert isinstance(self._right_front_lane_line_detection, lines), "Wrong type, should be 'lines'!"
		assert isinstance(self._left_rear_lane_line_detection, lines), "Wrong type, should be 'lines'!"
		assert isinstance(self._right_rear_lane_line_detection, lines), "Wrong type, should be 'lines'!"

		# left front
		if self._left_front_lane_line_detection.line_number > 0:
			ego_line_lf = None
			for i in range(self._left_front_lane_line_detection.line_number):
				if self._left_front_lane_line_detection.all_lines[i].id == -1:
					ego_line_lf = self._left_front_lane_line_detection.all_lines[i]
					break

			if ego_line_lf != None and ego_line_lf.score > prob_threshold:
				self._left_front_ego_lane_line['positionx'] = np.array(ego_line_lf.positionx)
				self._left_front_ego_lane_line['positiony'] = np.array(ego_line_lf.positiony)
				self._is_left_front_ego_lane_detected = True
			else:
				self._left_front_ego_lane_line['positionx'] = np.array([])
				self._left_front_ego_lane_line['positiony'] = np.array([])
				self._is_left_front_ego_lane_detected = False
		else:
			self._left_front_ego_lane_line['positionx'] = np.array([])
			self._left_front_ego_lane_line['positiony'] = np.array([])
			self._is_left_front_ego_lane_detected = False

		#print("left_front: {}".format(self._left_front_ego_lane_line['positionx']))


		# right front
		if self._right_front_lane_line_detection.line_number > 0:
			ego_line_rf = None
			for i in range(self._right_front_lane_line_detection.line_number):
				if self._right_front_lane_line_detection.all_lines[i].id == 1:
					ego_line_rf = self._right_front_lane_line_detection.all_lines[i]
					break

			if ego_line_rf != None and ego_line_rf.score > prob_threshold:
				self._right_front_ego_lane_line['positionx'] = np.array(ego_line_rf.positionx)
				self._right_front_ego_lane_line['positiony'] = np.array(ego_line_rf.positiony)
				self._is_right_front_ego_lane_detected = True
			else:
				self._right_front_ego_lane_line['positionx'] = np.array([])
				self._right_front_ego_lane_line['positiony'] = np.array([])
				self._is_right_front_ego_lane_detected = False
		else:
			self._right_front_ego_lane_line['positionx'] = np.array([])
			self._right_front_ego_lane_line['positiony'] = np.array([])
			self._is_right_front_ego_lane_detected = False

		#print("right_front: {}".format(self._right_front_ego_lane_line['positionx']))

		prob_threshold = 0.92 
		# left rear
		if self._left_rear_lane_line_detection.line_number > 0:
			ego_line_lr = None
			for i in range(self._left_rear_lane_line_detection.line_number):
				if self._left_rear_lane_line_detection.all_lines[i].id == 1:
					ego_line_lr = self._left_rear_lane_line_detection.all_lines[i]
					break

			if ego_line_lr != None and ego_line_lr.score > prob_threshold:
				self._left_rear_ego_lane_line['positionx'] = np.array(ego_line_lr.positionx)
				self._left_rear_ego_lane_line['positiony'] = np.array(ego_line_lr.positiony)
				self._is_left_rear_ego_lane_detected = True
			else:
				self._left_rear_ego_lane_line['positionx'] = np.array([])
				self._left_rear_ego_lane_line['positiony'] = np.array([])
				self._is_left_rear_ego_lane_detected = False
		else:
			self._left_rear_ego_lane_line['positionx'] = np.array([])
			self._left_rear_ego_lane_line['positiony'] = np.array([])
			self._is_left_rear_ego_lane_detected = False

		#print("left_rear: {}".format(self._left_rear_ego_lane_line['positionx']))
			

		# right rear
		if self._right_rear_lane_line_detection.line_number > 0:
			ego_line_rr = None
			for i in range(self._right_rear_lane_line_detection.line_number):
				if self._right_rear_lane_line_detection.all_lines[i].id == -1:
					ego_line_rr = self._right_rear_lane_line_detection.all_lines[i]
					break

			if ego_line_rr != None and ego_line_rr.score > prob_threshold:
				self._right_rear_ego_lane_line['positionx'] = np.array(ego_line_rr.positionx)
				self._right_rear_ego_lane_line['positiony'] = np.array(ego_line_rr.positiony)
				self._is_right_rear_ego_lane_detected = True
			else:
				self._right_rear_ego_lane_line['positionx'] = np.array([])
				self._right_rear_ego_lane_line['positiony'] = np.array([])
				self._is_right_rear_ego_lane_detected = False
		else:
			self._right_rear_ego_lane_line['positionx'] = np.array([])
			self._right_rear_ego_lane_line['positiony'] = np.array([])
			self._is_right_rear_ego_lane_detected = False

		#print("right_rear: {}".format(self._right_rear_ego_lane_line['positionx']))
				

	def update_intercepts_with_ego_lane_lines(self):
		# a module
		def _calc_intercept(x_list, y_list, ref_func, polyfit_solver, intercept_solver, min_num_points=4, start=0.0):
			# delete zero
			del_list = []
			# non-zero data points
			data_len = x_list.shape[0]
			if data_len > 0:
				for i in range(data_len):
					#if x_list[i]==y_list[i]==0 or x_list[i]>self.CAM_WIDTH-1 or x_list[i]<0 or y_list[i]<0 or y_list[i]>self.CAM_HEIGHT:
					if x_list[i]==y_list[i]==0:
						del_list.append(i)
				x_list = np.delete(x_list, del_list)
				y_list = np.delete(y_list, del_list)
				
				if x_list.shape[0]==0:
					return np.array([])

				# check order
				if x_list[0] > x_list[-1]:
					x_list = x_list[::-1]
					y_list = y_list[::-1]
				# x, y1, y2
				if data_len < min_num_points:
					# select domain of x, evenly distributed, it takes longer time!
					x_domain = np.linspace(x_list[0], x_list[-1], 10)
					# calc y values of ref line corresponding to domain of x
					y_ref = ref_func(x_domain)
					# fit y_list
					y_func = polyfit_solver(x_list, y_list, 2)
					# calc y values corresponding to domain of x
					new_y_list = y_func(x_domain)
					# limit start
					if start < x_domain[0]:
						start = x_domain[0]
					elif start > x_domain[-1]:
						start = x_domain[-1]
					# calc intercept
					x_cross, _, _ = intercept_solver(x_domain, y_ref, new_y_list, start)
					y_cross = ref_func(x_cross)
					# return intercept
					return np.array([x_cross, y_cross])
				else:
					# it is reasonable to take x_list as evenly distributed
					x_domain = x_list
					# remove dirty data
					del_list = []
					for i, x in enumerate(x_domain):
						if i > 0:
							if last_x > x:
								del_list.append(i)
							else:
								last_x = x
						else:
							last_x = x

					x_domain = np.delete(x_domain, del_list)
					y_list = np.delete(y_list, del_list)

					#print('x_domain: ', x_domain)
					# calc y values of ref line corresponding to domain of x
					y_ref = ref_func(x_domain)
					# calc y values of left front ego lane line corresponding to domain of x
					new_y_list = y_list
					# limit start
					if start < x_domain[0]:
						start = x_domain[0]
					elif start > x_domain[-1]:
						start = x_domain[-1]
					# calc intercept
					x_cross, _, _ = intercept_solver(x_domain, y_ref, new_y_list, start)
					y_cross = ref_func(x_cross) 
					# return intercept
					return np.array([x_cross[0], y_cross[0]])
			else:
				return np.array([])


		minimum_points = 4 # got from trial and error
		##############
		# left front
		##############
		pos_x_lf = self._left_front_ego_lane_line['positionx'] / self.CAM_WIDTH
		pos_y_lf = self._left_front_ego_lane_line['positiony'] / self.CAM_HEIGHT
		x_start = pos_x_lf[-1] if pos_x_lf.shape[0] > 0 else 0.0
		self._left_front_intercept_with_ego_lane_line = _calc_intercept(pos_x_lf, pos_y_lf, self._left_front_ref_func, polyfit_solver, intercept_solver, min_num_points=minimum_points, start=x_start)

		#print(self._left_front_intercept_with_ego_lane_line)
		##############
		# right front
		##############
		pos_x_rf = self._right_front_ego_lane_line['positionx'] / self.CAM_WIDTH
		pos_y_rf = self._right_front_ego_lane_line['positiony'] / self.CAM_HEIGHT
		x_start = pos_x_rf[0] if pos_x_rf.shape[0] > 0 else 0.0
		self._right_front_intercept_with_ego_lane_line = _calc_intercept(pos_x_rf, pos_y_rf, self._right_front_ref_func, polyfit_solver, intercept_solver, min_num_points=minimum_points, start=x_start)

		#print(self._right_front_intercept_with_ego_lane_line)
		#############
		# left rear 
		#############
		pos_x_lr = self._left_rear_ego_lane_line['positionx'] / self.CAM_WIDTH
		pos_y_lr = self._left_rear_ego_lane_line['positiony'] / self.CAM_HEIGHT
		x_start = pos_x_lr[0] if pos_x_lr.shape[0] > 0 else 0.0
		self._left_rear_intercept_with_ego_lane_line = _calc_intercept(pos_x_lr, pos_y_lr, self._left_rear_ref_func, polyfit_solver, intercept_solver, min_num_points=minimum_points, start=x_start)

		#print(self._left_rear_intercept_with_ego_lane_line)
		#############
		# right rear
		#############
		pos_x_rr = self._right_rear_ego_lane_line['positionx'] / self.CAM_WIDTH
		pos_y_rr = self._right_rear_ego_lane_line['positiony'] / self.CAM_HEIGHT
		x_start = pos_x_rr[-1] if pos_x_rr.shape[0] > 0 else 0.0
		self._right_rear_intercept_with_ego_lane_line = _calc_intercept(pos_x_rr, pos_y_rr, self._right_rear_ref_func, polyfit_solver, intercept_solver, min_num_points=minimum_points, start=x_start)
		#print(self._right_rear_intercept_with_ego_lane_line)		
		#print("********************************")

	def update_neighbor_lane_line(self):
		# do some filtering thing
		pass

	def update_intercept_with_neighbor_lane_line(self):
		pass

	def update_all_distances_to_ego_lane_lines(self, c_speed, c_dev_angle, delta_t, maneuvering_type='lane_keep'):
		'''
		c_speed:		current vehicle speed
		c_dev_angle:	current angle between vehicle and lane
		delta_t:		time cost of each step
		'''
		# left front
		if (self._left_front_intercept_with_ego_lane_line).shape[0] == 2:
			pix_x, pix_y = self._left_front_intercept_with_ego_lane_line
			self._distance_from_head_to_left_front_ego_lane_line, _ = estimate_real_location(pix_x, pix_y, self._left_front_pix_real_pos_map)
			# FIXME: due to calibration
			self._distance_from_head_to_left_front_ego_lane_line = -self._distance_from_head_to_left_front_ego_lane_line
			#print("lf: ", self._distance_from_head_to_left_front_ego_lane_line)
		else:
			self._distance_from_head_to_left_front_ego_lane_line = None

		# right front
		if (self._right_front_intercept_with_ego_lane_line).shape[0] == 2:
			pix_x, pix_y = self._right_front_intercept_with_ego_lane_line
			self._distance_from_head_to_right_front_ego_lane_line, _ = estimate_real_location(pix_x, pix_y, self._right_front_pix_real_pos_map)
			#print("rf: ", self._distance_from_head_to_right_front_ego_lane_line)
		else:
			self._distance_from_head_to_right_front_ego_lane_line = None

		# left rear
		if (self._left_rear_intercept_with_ego_lane_line).shape[0] == 2:
			pix_x, pix_y = self._left_rear_intercept_with_ego_lane_line
			self._distance_from_wheel_to_left_rear_ego_lane_line, _ = estimate_real_location(pix_x, pix_y, self._left_rear_pix_real_pos_map)
			#print("lr: ", self._distance_from_wheel_to_left_rear_ego_lane_line)
		else:
			self._distance_from_wheel_to_left_rear_ego_lane_line = None
		
		# right rear
		if (self._right_rear_intercept_with_ego_lane_line).shape[0] == 2:
			pix_x, pix_y = self._right_rear_intercept_with_ego_lane_line
			self._distance_from_wheel_to_right_rear_ego_lane_line, _ = estimate_real_location(pix_x, pix_y, self._right_rear_pix_real_pos_map)
			#print("rr: ", self._distance_from_wheel_to_right_rear_ego_lane_line)
		else:
			self._distance_from_wheel_to_right_rear_ego_lane_line = None
		#print("********************************")

		#####################################
		# size of window of historical data
		window_size = 10 # FIXME

		def _next_data_approximation(historical_data, current_dev_angle, current_speed, t):
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
				approximate_0 = a
				approximate_1 = a + current_speed*t*np.tan(current_dev_angle*np.pi/180.0)
				# expected
				return 0.5*(approximate_0 + approximate_1)
			else:
				return None
	
			'''
			if len(historical_data) > 0:
				return historical_data[-1]
			else:
				return None
			'''

		def _outlier_filter(approximate, measured):
			epsilon = 0.00000001
			threshold = 0.5
			if measured == None:
				return approximate
			else:
				if approximate != None:
					if np.abs(approximate - measured)/(approximate+epsilon) <= threshold:
						return measured
					else:
						return approximate
				else:
					return measured

		##
		next_approximate_distance_left_front = _next_data_approximation(self._historical_distances_from_head_to_left_front_ego_lane_line)
		next_approximate_distance_right_front = _next_data_approximation(self._historical_distances_from_head_to_right_front_ego_lane_line)
		next_approximate_distance_left_rear = _next_data_approximation(self._historical_distances_from_wheel_to_left_rear_ego_lane_line)
		next_approximate_distance_right_rear = _next_data_approximation(self._historical_distances_from_wheel_to_right_rear_ego_lane_line)

		if maneuvering_type == 'lane_keep':
			## lf
			if self._distance_from_head_to_left_front_ego_lane_line == None:
				self._distance_from_head_to_left_front_ego_lane_line = next_approximate_distance_left_front
			else:
				# filtering left front
				filtered = _outlier_filter(next_approximate_distance_left_front, self._distance_from_head_to_left_front_ego_lane_line)
				self._distance_from_head_to_left_front_ego_lane_line = filtered

			self._historical_distances_from_head_to_left_front_ego_lane_line.append(self._distance_from_head_to_left_front_ego_lane_line)
			print("lf: ", self._distance_from_head_to_left_front_ego_lane_line)

			## rf
			if self._distance_from_head_to_right_front_ego_lane_line == None:
				self._distance_from_head_to_right_front_ego_lane_line = next_approximate_distance_right_front
			else:
				# filtering right front
				filtered = _outlier_filter(next_approximate_distance_right_front, self._distance_from_head_to_right_front_ego_lane_line)
				self._distance_from_head_to_right_front_ego_lane_line = filtered

			self._historical_distances_from_head_to_right_front_ego_lane_line.append(self._distance_from_head_to_right_front_ego_lane_line)
			print("rf: ", self._distance_from_head_to_right_front_ego_lane_line)

			## lr
			if self._distance_from_wheel_to_left_rear_ego_lane_line == None:
				self._distance_from_wheel_to_left_rear_ego_lane_line = next_approximate_distance_left_rear
			else:
				# filtering left rear
				filtered = _outlier_filter(next_approximate_distance_left_rear, self._distance_from_wheel_to_left_rear_ego_lane_line)
				self._distance_from_wheel_to_left_rear_ego_lane_line = filtered

			self._historical_distances_from_wheel_to_left_rear_ego_lane_line.append(self._distance_from_wheel_to_left_rear_ego_lane_line)
			print("lr: ", self._distance_from_wheel_to_left_rear_ego_lane_line)

			## rr
			if self._distance_from_wheel_to_right_rear_ego_lane_line == None:
				self._distance_from_wheel_to_right_rear_ego_lane_line = next_approximate_distance_right_rear
			else:
				# filtering right rear
				filtered = _outlier_filter(next_approximate_distance_right_rear, self._distance_from_wheel_to_right_rear_ego_lane_line)
				self._distance_from_wheel_to_right_rear_ego_lane_line = filtered

			self._historical_distances_from_wheel_to_right_rear_ego_lane_line.append(self._distance_from_wheel_to_right_rear_ego_lane_line)
			print("rr: ", self._distance_from_wheel_to_right_rear_ego_lane_line)
			print("************************")

			# length truncated to be sized 50
			if len(self._historical_distances_from_head_to_left_front_ego_lane_line) > window_size:
				self._historical_distances_from_head_to_left_front_ego_lane_line.pop(0)
				self._historical_distances_from_head_to_right_front_ego_lane_line.pop(0)
				self._historical_distances_from_wheel_to_left_rear_ego_lane_line.pop(0)
				self._historical_distances_from_wheel_to_right_rear_ego_lane_line.pop(0)

		elif maneuvering_type == 'lane_change':
			self._historical_distances_from_head_to_left_front_ego_lane_line = []
			self._historical_distances_from_head_to_right_front_ego_lane_line = []
			self._historical_distances_from_wheel_to_left_rear_ego_lane_line = []
			self._historical_distances_from_wheel_to_right_rear_ego_lane_line = []
		

	# off lane check	
	def off_lane_warning(self):
		pass
