import os, sys
import numpy as np


class PoseEstimator:
	TOWARDS_LEFT = -1
	NEUTRAL = 0
	TOWARDS_RIGHT = 1

	def __init__(self, dist_to_left_front_bounds, 
					   dist_to_right_front_bounds, 
					   dist_to_left_rear_bounds,
					   dist_to_right_rear_bounds,
					   wheel_base,
					   width,
					   dist_from_front_wheel_to_head,
					   dist_from_rear_wheel_to_tail):
		'''
		dist_to_left_front_bounds:  distance from left front wheel to left front bounds (left lane line)
		dist_to_right_front_bounds: distance from right front wheel to right front bounds (right lane line)
		dist_to_left_rear_bounds:   distance from left rear wheel to left rear bounds (left lane line)
		dist_to_right_rear_bounds:	distance from right rear wheel to right rear bounds (right lane line)
		wheel_base:					wheel base
		width: 						width of the vehicle
		dist_from_front_wheel_to_head:	distance from front wheel to head
		dist_from_rear_wheel_to_tail:	distance from rear wheel to tail
		'''
		# distance to bounds
		self._dist_to_left_front_bounds = dist_to_left_front_bounds
		self._dist_to_right_front_bounds = dist_to_right_front_bounds
		self._dist_to_left_rear_bounds = dist_to_left_rear_bounds
		self._dist_to_right_rear_bounds = dist_to_right_rear_bounds
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

	@property
	def dev_angle(self):
		return self._dev_angle

	@property
	def dev_direct(self):
		return self._dev_direct

	def __call__(self, dist_to_left_front_bounds,
					   dist_to_right_front_bounds,
					   dist_to_left_rear_bounds,
					   dist_to_right_rear_bounds):
		# update
		self._dist_to_left_front_bounds = dist_to_left_front_bounds
		self._dist_to_right_front_bounds = dist_to_right_front_bounds
		self._dist_to_left_rear_bounds = dist_to_left_rear_bounds
		self._dist_to_right_rear_bounds = dist_to_right_rear_bounds

	def __convert_to_vehicle_xoy(self):
		joint_xy_left_front_bounds = (-0.5*self._width-self._dist_to_left_front_bounds, self._wheel_base+self._dist_from_front_wheel_to_head)
		joint_xy_right_front_bounds = (0.5*self._width+self._dist_to_right_front_bounds, self._wheel_base+self._dist_from_front_wheel_to_head)
		joint_xy_left_rear_bounds = (-0.5*self._width-self._dist_to_left_rear_bounds, 0.0)
		joint_xy_right_rear_bounds = (0.5*self._width+self._dist_to_right_rear_bounds, 0.0)
		
		return [joint_xy_left_front_bounds, joint_xy_right_front_bounds, joint_xy_left_rear_bounds, joint_xy_right_rear_bounds]


	def estimate_pose(self):
		joint_xy_left_front_bounds, joint_xy_right_front_bounds, joint_xy_left_rear_bounds, joint_xy_right_rear_bounds = __convert_to_vehicle_xoy()

		lane_center_axis_front_end = (0.5*(joint_xy_left_front_bounds[0]+joint_xy_right_front_bounds[0]), 0.5*(joint_xy_left_front_bounds[1]+joint_xy_right_front_bounds[1]))
		lane_center_axis_rear_end = (0.5*(joint_xy_left_rear_bounds[0]+joint_xy_right_rear[0]), 0.5*(joint_xy_left_rear_bounds[1]+joint_xy_right_rear_bounds[1]))
		#
		front_end_x, front_end_y = lane_center_axis_front_end
		rear_end_x, rear_end_y = lane_center_axis_rear_end

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
			self._dev_angle = np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))
			self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
			self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
			self._pose_type = 3
			
		elif rear_end_x < 0.0 and front_end_x > 0.0:
			self._dev_direct = self.TOWARDS_LEFT
			self._dev_angle = np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))
			self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
			self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
			self._pose_type = 4

		elif front_end_x > rear_end_x >= 0.0:
			self._dev_direct = self.TOWARDS_LEFT
			self._dev_angle = np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))
			self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
			self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
			self._pose_type = 5

		elif front_end_x < rear_end_x <= 0.0:
			self._dev_direct = self.TOWARDS_RIGHT
			self._dev_angle = np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))
			self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
			self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
			self._pose_type = 6
			
		elif front_end_x < 0.0 and rear_end_x > 0.0:
			self._dev_direct = self.TOWARDS_RIGHT
			self._dev_angle = np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))
			self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
			self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
			self._pose_type = 7

		elif rear_end_x > front_end_x >= 0.0:
			self._dev_direct = self.TOWARDS_RIGHT
			self._dev_angle = np.arctan(np.abs(front_end_x-rear_end_x)/(self._wheel_base+self._dist_from_front_wheel_to_head))
			self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center = -front_end_x
			self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center = -rear_end_x
			self._pose_type = 8
			
		return  self._dev_direct, self._dev_angle, self._dist_on_x_axis_from_vehicle_front_end_center_to_lane_center, self._dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center, self._pose_type

