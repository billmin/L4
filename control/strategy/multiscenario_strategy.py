import os, sys
from fuzzy_control_reference import *


def lane_center_and_adaptive_cruise_control(lateral_deviation_from_lane_center, 
											ego_speed,
											distance_to_front_vehicle_or_pedestrian_in_ego_lane=None,
											speed_of_front_vehicle_or_pedestrian_in_ego_lane=None):
	pass


def lane_change_control(lateral_direction_to, 
						stage, 
						ego_speed, 
						distance_to_front_vehicle_in_origin_lane, 
						speed_of_front_vehicle_in_origin_lane,
						distance_to_front_vehicle_in_target_lane,
						speed_of_front_vehicle_in_target_lane,
						):
	pass


def travel_across_intersection(direction_to, stage, ego_speed, ):
