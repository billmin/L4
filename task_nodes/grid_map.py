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
from multi_task_msgs import WideSpeedRangeAdaptiveCruiseBasis
from multi_task_msgs import LaneChangeToLeftBasis
from multi_task_msgs import LaneChangeToRightBasis
from multi_task_msgs import TurnLeftAtCrossBasis
from multi_task_msgs import TurnRightAtCrossBasis
from multi_task_msgs import GoStraightAtCrossBasis
from multi_task_msgs import UTurnBasis
from multi_task_msgs import StopAtObstacleBasis
from multi_task_msgs import HaltBehindStopLineBasis
from multi_task_msgs import BypassObstacleOnLeftBasis
from multi_task_msgs import BypassObstacleOnRightBasis
from multi_task_msgs import OvertakeOnLeftBasis
from multi_task_msgs import OvertakeOnRightBasis
from multi_task_msgs import SlowDownForPedestrianTraverseBasis
from multi_task_msgs import TemporaryStopByRoadsideBasis

def callback_wide_speed_range_adaptive_cruise(data_perception):
	global wide_speed_range_adaptive_cruise_basis
	wide_speed_range_adaptive_cruise_basis = data_perception

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

def callback_halt_behind_stop_line(data_perception):
	global halt_behind_stop_line_basis
	halt_behind_stop_line_basis = data_perception

def callback_slow_down_for_pedestrian_traverse(data_perception):
	global slow_down_for_pedestrian_traverse_basis
	slow_down_for_pedestrian_traverse_basis = data_perception

def callback_bypass_obstacle_on_left(data_perception):
	global bypass_obstacle_on_left_basis
	bypass_obstacle_on_left_basis = data_perception

def callback_bypass_obstacle_on_right(data_perception):
	global bypass_obstacle_on_right_basis
	bypass_obstacle_on_right_basis = data_perception

def callback_overtake_on_left(data_perception):
	global overtake_on_left_basis
	overtake_on_left_basis = data_perception

def callback_overtake_on_right(data_perception):
	global overtake_on_right_basis
	overtake_on_right_basis = data_perception

def callback_temporary_stop_by_roadside(data_perception):
	global temporary_stop_by_roadside_basis
	temporary_stop_by_roadside_basis = data_perception

def listener():
	rospy.init_node("surrounding_build", anonymous=True)
	# subscribers

	rospy.Subscriber("/multi_task/wide_speed_range_adaptive_cruise", WideSpeedRangeAdaptiveCruiseBasis, callback_wide_speed_range_adaptive_cruise, queue_size=1)
	rospy.Subscriber("/multi_task/lane_change_to_left", LaneChangeToLeftBasis, callback_lane_change_to_left, queue_size=1)
	rospy.Subscriber("/multi_task/lane_change_to_right", LaneChangeToRightBasis, callback_lane_change_to_right, queue_size=1)
	rospy.Subscriber("/multi_task/turn_left_at_cross", TurnLeftAtCrossBasis, callback_turn_left_at_cross, queue_size=1)
	rospy.Subscriber("/multi_task/turn_right_at_cross", TurnRightAtCrossBasis, callback_turn_right_at_cross, queue_size=1)
	rospy.Subscriber("/multi_task/go_straight_at_cross", GoStraightAtCrossBasis, callback_go_straight_at_cross, queue_size=1)
	rospy.Subscriber("/multi_task/u_turn", UTurnBasis, callback_u_turn, queue_size=1)
	rospy.Subscriber("/multi_task/stop_at_obstacle", StopAtObstacleBasis, callback_stop_at_obstacle, queue_size=1)
	rospy.Subscriber("/multi_task/halt_behind_stop_line", HaltBehindStopLineBasis, callback_halt_behind_stop_line, queue_size=1)
	rospy.Subscriber("/multi_task/slow_down_for_pedestrian_traverse", SlowDownForPedestrianTraverseBasis, callback_slow_down_for_pedestrian_traverse, queue_size=1)
	rospy.Subscriber("/multi_task/bypass_obstacle_on_left", BypassObstacleOnLeftBasis, callback_bypass_obstacle_on_left, queue_size=1)
	rospy.Subscriber("/multi_task/bypass_obstacle_on_right", BypassObstacleOnRightBasis, callback_bypass_obstacle_on_right, queue_size=1)
	rospy.Subscriber("/multi_task/overtake_on_left", OvertakeOnLeftBasis, callback_overtake_on_left, queue_size=1)
	rospy.Subscriber("/multi_task/overtake_on_right", OvertakeOnRightBasis, callback_overtake_on_right, queue_size=1)
	rospy.Subscriber("/multi_task/temporary_stop_by_roadside", TemporaryStopByRoadsideBasis, callback_temporary_stop_by_roadside, queue_size=1)

	# publishers
	pub_surrounding_build = rospy.Publisher("/multi_task/surrounding_build", SurroundingBuild, queue_size=1)

	rate = rospy.Rate(50)

	# decision basis for lane keep
	global wide_speed_range_adaptive_cruise_basis
	wide_speed_range_adaptive_cruise_basis = WideSpeedRangeAdaptiveCruiseBasis()
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
	# decision basis for halting behind stop line
	global halt_behind_stop_line_basis
	halt_behind_stop_line_basis = HaltBehindStopLineBasis()
	# slow down for pedestrian
	global slow_down_for_pedestrian_traverse_basis
	slow_down_for_pedestrian_traverse_basis = SlowDownForPedestrianTraverseBasis()
	# decision basis for bypassing obstacle on left
	global bypass_obstacle_on_left_basis
	bypass_obstacle_on_left_basis = BypassObstacleOnLeftBasis()
	# decision basis for bypassing obstacle on right
	global bypass_obstacle_on_right_basis
	bypass_obstacle_on_right_basis = BypassObstacleOnRightBasis()
	# decision basis for overtaking on left
	global overtake_on_left_basis
	overtake_on_left_basis = OvertakeOnLeftBasis()
	# decision basis for overtaking on right
	global overtake_on_right_basis
	overtake_on_right_basis = OvertakeOnRightBasis()
	# decision basis for temporary stop by the roadside
	global temporary_stop_by_roadside_basis
	temporary_stop_by_roadside_basis = TemporaryStopByRoadsideBasis()

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



