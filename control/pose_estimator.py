import os, sys
import numpy as np


class PoseEstimator:
	TOWARDS_LEFT = -1
	NEUTRAL = 0
	TOWARDS_RIGHT = 1

	def __init__(self, dist_to_left_front_ego_lane_line, 
					   dist_to_right_front_ego_lane_line, 
					   dist_to_left_rear_ego_lane_line,
					   dist_to_right_rear_ego_lane_line,
					   wheel_base,
					   width,
					   dist_from_front_wheel_to_head,
					   dist_from_rear_wheel_to_tail):
		'''
		dist_to_left_front_ego_lane_line:  distance from left front wheel to left front ego_lane_line (left lane line)
		dist_to_right_front_ego_lane_line: distance from right front wheel to right front ego_lane_line (right lane line)
		dist_to_left_rear_ego_lane_line:   distance from left rear wheel to left rear ego_lane_line (left lane line)
		dist_to_right_rear_ego_lane_line:	distance from right rear wheel to right rear ego_lane_line (right lane line)
		wheel_base:					wheel base
		width: 						width of the vehicle
		dist_from_front_wheel_to_head:	distance from front wheel to head
		dist_from_rear_wheel_to_tail:	distance from rear wheel to tail
		'''
		# distance to ego_lane_line
		self._dist_to_left_front_ego_lane_line = dist_to_left_front_ego_lane_line
		self._dist_to_right_front_ego_lane_line = dist_to_right_front_ego_lane_line
		self._dist_to_left_rear_ego_lane_line = dist_to_left_rear_ego_lane_line
		self._dist_to_right_rear_ego_lane_line = dist_to_right_rear_ego_lane_line
		# is ego lane line detected 
		self._is_left_front_ego_lane_line_detected = is_left_front_ego_lane_line_detected
		self._is_right_front_ego_lane_line_detected = is_right_front_ego_lane_line_detected
		self._is_left_rear_ego_lane_line_detected = is_left_rear_ego_lane_line_detected
		self._is_right_rear_ego_lane_line_detected = is_right_rear_ego_lane_line_detected
		# deviation angle
		self._dev_angle = 0.0
		# deviation direct relative to lane heading direction
		self._dev_direct = self.NEUTRAL
		# distance from vehicle center axis to lane center axis
		self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = 0.0
		self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = 0.0
		# vehicle data
		self._wheel_base = wheel_base
		self._width = width
		self._pose_type = 0
		self._dist_from_front_wheel_to_head = dist_from_front_wheel_to_head
		self._dist_from_rear_wheel_to_tail = dist_from_rear_wheel_to_tail
		# whether to tell controller to output the same U of last moment
		self._use_last_u = False

	@property
	def dev_angle(self):
		return self._dev_angle

	@property
	def dev_direct(self):
		return self._dev_direct

	def __call__(self, dist_to_left_front_ego_lane_line,
					   dist_to_right_front_ego_lane_line,
					   dist_to_left_rear_ego_lane_line,
					   dist_to_right_rear_ego_lane_line):
		# update
		self._dist_to_left_front_ego_lane_line = dist_to_left_front_ego_lane_line
		self._dist_to_right_front_ego_lane_line = dist_to_right_front_ego_lane_line
		self._dist_to_left_rear_ego_lane_line = dist_to_left_rear_ego_lane_line
		self._dist_to_right_rear_ego_lane_line = dist_to_right_rear_ego_lane_line

	def __convert_to_vehicle_xoy(self):
		intercept_with_left_front_ego_lane_line = (-0.5*self._width-self._dist_to_left_front_ego_lane_line, self._wheel_base+self._dist_from_front_wheel_to_head)
		intercept_with_right_front_ego_lane_line = (0.5*self._width+self._dist_to_right_front_ego_lane_line, self._wheel_base+self._dist_from_front_wheel_to_head)
		intercept_with_left_rear_ego_lane_line = (-0.5*self._width-self._dist_to_left_rear_ego_lane_line, 0.0)
		intercept_with_right_rear_ego_lane_line = (0.5*self._width+self._dist_to_right_rear_ego_lane_line, 0.0)
		
		return intercept_with_left_front_ego_lane_line, intercept_with_right_front_ego_lane_line, intercept_with_left_rear_ego_lane_line, intercept_with_right_rear_ego_lane_line


	def estimate_pose(self):
		coe_rad_deg = 180.0/np.pi

		#############################################
		# only two distances of left side are valid
		#############################################
		if (self._dist_to_left_front_ego_lane_line!=None) and (self._dist_to_left_rear_ego_lane_line!=None) and (self._dist_to_right_front_ego_lane_line==None) and (self._dist_to_right_rear_ego_lane_line==None):
			if self._dist_to_left_front_ego_lane_line == self._dist_to_left_rear_ego_lane_line:
				self._dev_direct = self.NEUTRAL
				self._dev_angle = 0.0
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = None
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = None
				self._pose_type = 9
			elif self._dist_to_left_front_ego_lane_line < self._dist_to_left_rear_ego_lane_line:
				self._dev_direct = self.TOWARDS_LEFT
				self._dev_angle = -np.arctan(np.abs(self._dist_to_left_front_ego_lane_line-self._dist_to_left_rear_ego_lane_line)/(self._wheel_base+self._dist_from_front_wheel_to_head))*coe_rad_deg
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = None
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = None
				self._pose_type = 10
			else:
				self._dev_direct = self.TOWARDS_RIGHT
				self._dev_angle = np.arctan(np.abs(self._dist_to_left_front_ego_lane_line-self._dist_to_left_rear_ego_lane_line)/(self._wheel_base+self._dist_from_front_wheel_to_head))*coe_rad_deg
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = None
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = None
				self._pose_type = 11

			# update use_last_u flag
			self._use_last_u = False
		
		#############################################
		# only two distances of right side are valid
		#############################################
		elif (self._dist_to_left_front_ego_lane_line==None) and (self._dist_to_left_rear_ego_lane_line==None) and (self._dist_to_right_front_ego_lane_line!=None) and (self._dist_to_right_rear_ego_lane_line!=None):
			if self._dist_to_right_front_ego_lane_line == self._dist_to_right_rear_ego_lane_line:
				self._dev_direct = self.NEUTRAL
				self._dev_angle = 0.0
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = None
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = None
				self._pose_type = 12
			elif self._dist_to_right_front_ego_lane_line < self._dist_to_right_rear_ego_lane_line:
				self._dev_direct = self.TOWARDS_LEFT
				self._dev_angle = -np.arctan(np.abs(self._dist_to_right_front_ego_lane_line-self._dist_to_right_rear_ego_lane_line)/(self._wheel_base+self._dist_from_front_wheel_to_head))*coe_rad_deg
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = None
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = None
				self._pose_type = 13
			else:
				self._dev_direct = self.TOWARDS_RIGHT
				self._dev_angle = np.arctan(np.abs(self._dist_to_right_front_ego_lane_line-self._dist_to_right_rear_ego_lane_line)/(self._wheel_base+self._dist_from_front_wheel_to_head))*coe_rad_deg
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = None
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = None
				self._pose_type = 14

			# update use_last_u flag
			self._use_last_u = False
			
		################################
		# all four distances are valid
		################################
		elif (self._dist_to_left_front_ego_lane_line!=None) and (self._dist_to_left_rear_ego_lane_line!=None) and (self._dist_to_right_front_ego_lane_line!=None) and (self._dist_to_right_rear_ego_lane_line!=None):
			intercept_with_left_front_ego_lane_line, intercept_with_right_front_ego_lane_line, intercept_with_left_rear_ego_lane_line, intercept_with_right_rear_ego_lane_line = self.__convert_to_vehicle_xoy()
			lane_center_coordinate_of_front_end = (0.5*(intercept_with_left_front_ego_lane_line[0]+intercept_with_right_front_ego_lane_line[0]), 0.5*(intercept_with_left_front_ego_lane_line[1]+intercept_with_right_front_ego_lane_line[1]))
			lane_center_coordinate_of_rear_end = (0.5*(intercept_with_left_rear_ego_lane_line[0]+intercept_with_right_rear_ego_lane_line[0]), 0.5*(intercept_with_left_rear_ego_lane_line[1]+intercept_with_right_rear_ego_lane_line[1]))
			#
			front_end_x, front_end_y = lane_center_coordinate_of_front_end
			rear_end_x, rear_end_y = lane_center_coordinate_of_rear_end

			# 9 cases
			if front_end_x == rear_end_x == 0:
				self._dev_direct = self.NEUTRAL
				self._dev_angle = 0.0
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = 0.0
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = 0.0
				self._pose_type = 0

			elif front_end_x == rear_end_x < 0.0:
				self._dev_direct = self.NEUTRAL
				self._dev_angle = 0.0
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
				self._pose_type = 1
	
			elif front_end_x == rear_end_x > 0.0:
				self._dev_direct = self.NEUTRAL
				self._dev_angle = 0.0
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
				self._pose_type = 2

			elif rear_end_x < front_end_x <= 0.0:
				self._dev_direct = self.TOWARDS_LEFT
				self._dev_angle = -np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))*coe_rad_deg
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
				self._pose_type = 3
			
			elif rear_end_x < 0.0 and front_end_x > 0.0:
				self._dev_direct = self.TOWARDS_LEFT
				self._dev_angle = -np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))*coe_rad_deg
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
				self._pose_type = 4

			elif front_end_x > rear_end_x >= 0.0:
				self._dev_direct = self.TOWARDS_LEFT
				self._dev_angle = -np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))*coe_rad_deg
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
				self._pose_type = 5

			elif front_end_x < rear_end_x <= 0.0:
				self._dev_direct = self.TOWARDS_RIGHT
				self._dev_angle = np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))*coe_rad_deg
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
				self._pose_type = 6
			
			elif front_end_x < 0.0 and rear_end_x > 0.0:
				self._dev_direct = self.TOWARDS_RIGHT
				self._dev_angle = np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))*coe_rad_deg
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
				self._pose_type = 7

			elif rear_end_x > front_end_x >= 0.0:
				self._dev_direct = self.TOWARDS_RIGHT
				self._dev_angle = np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))*coe_rad_deg
				self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
				self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
				self._pose_type = 8

			# update use_last_u flag
			self._use_last_u = False
		
		################
		## other cases
		################
		else:
			# these are not keys
			self._dev_direct = None
			self._dev_angle = None
			self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = None
			self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = None
			self._pose_type = None

			# this is key
			# update use_last_u flag
			self._use_last_u = True
			
			
		return  self._dev_direct, self._dev_angle, self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center, self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center, self._pose_type, self._use_last_u

