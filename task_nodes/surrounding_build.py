#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from multi_task_msgs import SurroundingBuild
from multi_task_msgs import LaneKeepBasis
from multi_task_msgs import LaneChangeToLeftBasis
from multi_task_msgs import LaneChangeToRightBasis
from multi_task_msgs import TurnLeftAtCrossBasis
from multi_task_msgs import TurnRightAtCrossBasis
from multi_task_msgs import GoStraightAtCrossBasis
from multi_task_msgs import UTurnBasis
from multi_task_msgs import StopAtObstacleBasis
from multi_task_msgs import StopAtRedLightBasis
from multi_task_msgs import BypassObstacleFromLeftBasis
from multi_task_msgs import BypassObstacleFromRightBasis
from multi_task_msgs import OvertakeFromLeftBasis
from multi_task_msgs import OvertakeFromRightBasis
from multi_task_msgs import CarFollowInCityBasis
from multi_task_msgs import CruiseAtHighSpeedBasis
from multi_task_msgs import TemporaryStopByRoadsideBasis

def callback_lane_keep(data_perception):
	global lane_keep_basis
	lane_keep_basis = data_perception

def callback_lane_change_to_left(data_perception):
	global lane_change_to_left_basis
	lane_change_to_left_basis = data_perception

def callback_lane_change_to_right(data_perception):
	global lane_change_to_right_basis
	lane_change_to_right_basis = data_perception

def callback_turn_left_at_cross(data_perception):
	global turn_left_at_cross_basis
	turn_left_at_cross_basis = data_perception

def callback_turn_right_at_cross(data_perception):
	global turn_right_at_cross_basis
	turn_right_at_cross_basis = data_perception

def callback_go_straight_at_cross(data_perception):
	global go_straight_at_cross_basis
	go_straight_at_cross_basis = data_perception

def callback_u_turn(data_perception):
	global u_turn_basis
	u_turn_basis = data_perception

def callback_stop_at_obstacle(data_perception):
	global stop_at_obstacle_basis
	stop_at_obstacle_basis = data_perception

def callback_stop_at_red_light(data_perception):
	global stop_at_red_light_basis
	stop_at_red_light_basis = data_perception

def callback_bypass_obstacle_from_left(data_perception):
	global bypass_obstacle_from_left_basis
	bypass_obstacle_from_left_basis = data_perception

def callback_bypass_obstacle_from_right(data_perception):
	global bypass_obstacle_from_right_basis
	bypass_obstacle_from_right_basis = data_perception

def callback_overtake_from_left(data_perception):
	global overtake_from_left_basis
	overtake_from_left_basis = data_perception

def callback_overtake_from_right(data_perception):
	global overtake_from_right_basis
	overtake_from_right_basis = data_perception

def callback_car_follow_in_city(data_perception):
	global car_follow_in_city_basis
	car_follow_in_city_basis = data_perception

def callback_cruise_at_high_speed(data_perception):
	global cruise_at_high_speed_basis
	cruise_at_high_speed_basis = data_perception

def callback_temporary_stop_by_roadside(data_perception):
	global temporary_stop_by_roadside_basis
	temporary_stop_by_roadside_basis = data_perception

def listener():
	rospy.init_node("surrounding_build", anonymous=True)
	# subscribers

	rospy.Subscriber("/multi_task/lane_keep", LaneKeepBasis, callback_lane_keep, queue_size=1)
	rospy.Subscriber("/multi_task/lane_change_to_left", LaneChangeToLeftBasis, callback_lane_change_to_left, queue_size=1)
	rospy.Subscriber("/multi_task/lane_change_to_right", LaneChangeToRightBasis, callback_lane_change_to_right, queue_size=1)
	rospy.Subscriber("/multi_task/turn_left_at_cross", TurnLeftAtCrossBasis, callback_turn_left_at_cross, queue_size=1)
	rospy.Subscriber("/multi_task/turn_right_at_cross", TurnRightAtCrossBasis, callback_turn_right_at_cross, queue_size=1)
	rospy.Subscriber("/multi_task/go_straight_at_cross", GoStraightAtCrossBasis, callback_go_straight_at_cross, queue_size=1)
	rospy.Subscriber("/multi_task/u_turn", UTurnBasis, callback_u_turn, queue_size=1)
	rospy.Subscriber("/multi_task/stop_at_obstacle", StopAtObstacleBasis, callback_stop_at_obstacle, queue_size=1)
	rospy.Subscriber("/multi_task/stop_at_red_light", StopAtRedLightBasis, callback_stop_at_red_light, queue_size=1)
	rospy.Subscriber("/multi_task/bypass_obstacle_from_left", BypassObstacleFromLeftBasis, callback_bypass_obstacle_from_left, queue_size=1)
	rospy.Subscriber("/multi_task/bypass_obstacle_from_right", BypassObstacleFromRightBasis, callback_bypass_obstacle_from_right, queue_size=1)
	rospy.Subscriber("/multi_task/overtake_from_left", OvertakeFromLeftBasis, callback_overtake_from_left, queue_size=1)
	rospy.Subscriber("/multi_task/overtake_from_right", OvertakeFromRightBasis, callback_overtake_from_right, queue_size=1)
	rospy.Subscriber("/multi_task/car_follow_in_city", CarFollowInCityBasis, callback_car_follow_in_city, queue_size=1)
	rospy.Subscriber("/multi_task/cruise_at_high_speed", CruiseAtHighSpeedBasis, callback_cruise_at_high_speed, queue_size=1)
	rospy.Subscriber("/multi_task/temporary_stop_by_roadside", TemporaryStopByRoadsideBasis, callback_temporary_stop_by_roadside, queue_size=1)

	# publishers
	pub_surrounding_build = rospy.Publisher("/multi_task/surrounding_build", SurroundingBuild, queue_size=1)

	rate = rospy.Rate(50)

	# decision basis for lane keep
	global lane_keep_basis
	lane_keep_basis = LaneKeepBasis()
	# decision basis for lane change to left
	global lane_change_to_left_basis
	lane_change_to_left_basis = LaneChangeToLeftBasis()
	# decision basis for lane change to right
	global lane_change_to_right_basis
	lane_change_to_right_basis = LaneChangeToRightBasis()
	# decision basis for turning left at cross
	global turn_left_at_cross_basis
	turn_left_at_cross_basis = TurnLeftAtCrossBasis()
	# decision basis for turning right at cross
	global turn_right_at_cross_basis
	turn_right_at_cross_basis = TurnRightAtCrossBasis()
	# decision basis for going straight at cross
	global go_straight_at_cross_basis
	go_straight_at_cross_basis = GoStraightAtCrossBasis()
	# decision basis for u turn
	global u_turn_basis
	u_turn_basis = UTurnBasis()
	# decision basis for stopping at obstacle
	global stop_at_obstacle_basis
	stop_at_obstacle_basis = StopAtObstacleBasis()
	# decision basis for stopping at red light
	global stop_at_red_light_basis
	stop_at_red_light_basis = StopAtRedLightBasis()
	# decision basis for bypassing obstacle from left
	global bypass_obstacle_from_left_basis
	bypass_obstacle_from_left_basis = BypassObstacleFromLeftBasis()
	# decision basis for bypassing obstacle from right
	global bypass_obstacle_from_right_basis
	bypass_obstacle_from_right_basis = BypassObstacleFromRightBasis()
	# decision basis for overtaking from left
	global overtake_from_left_basis
	overtake_from_left_basis = OvertakeFromLeftBasis()
	# decision basis for overtaking from right
	global overtake_from_right_basis
	overtake_from_right_basis = OvertakeFromRightBasis()
	# decision basis for car following in city
	global car_follow_in_city_basis
	car_follow_in_city_basis = CarFollowInCityBasis()
	# decision basis for cruise at high speed
	global cruise_at_high_speed_basis
	cruise_at_high_speed_basis = CruiseAtHighSpeedBasis()
	# decision basis for temporary stop by the roadside
	global temporary_stop_by_roadside_basis
	temporary_stop_by_roadside_basis = TemporaryStopByRoadsideBasis()

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



