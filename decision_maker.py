#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os

# recieved messages
from navi_msg import NaviCommand
from decision_basis_msgs.msgs import SurroundingBuild
from decision_basis_msgs.msgs import LaneKeepBasis
from decision_basis_msgs.msgs import LaneChangeToLeftBasis
from decision_basis_msgs.msgs import LaneChangeToRightBasis
from decision_basis_msgs.msgs import TurnLeftAtCrossBasis
from decision_basis_msgs.msgs import TurnRightAtCrossBasis
from decision_basis_msgs.msgs import GoStraightAtCrossBasis
from decision_basis_msgs.msgs import UTurnBasis
from decision_basis_msgs.msgs import StopAtObstacleBasis
from decision_basis_msgs.msgs import StopAtRedLightBasis
from decision_basis_msgs.msgs import BypassObstacleFromLeftBasis
from decision_basis_msgs.msgs import BypassObstacleFromRightBasis
from decision_basis_msgs.msgs import OvertakeFromLeftBasis
from decision_basis_msgs.msgs import OvertakeFromRightBasis
from decision_basis_msgs.msgs import CarFollowInCityBasis
from decision_basis_msgs.msgs import CruiseAtHighSpeedBasis
from decision_basis_msgs.msgs import TemporaryStopByRoadsideBasis

# published messages
from control_msgs.msgs import XControlStrategy, YControlStrategy

def callback_navigation(data_navi):
	global navi_command
	navi_command = data_navi

def callback_surrounding_build(data_percetions):
	global surrounding
	surrounding = data_percetions

# ------ basic tasks ------
# 1. lane keep
# 2. lane change to left
# 3. lane change to right
# 4. turn left at cross
# 5. turn right at cross
# 6. go straight at cross
# 7. u turn
# 8. stop at obstacle
# 9. stop at red light
# 10. bypass obstacle from left
# 11. bypass obstacle from right
# 12. overtake from left
# 13. overtake from right
# 14. car follow in city
# 15. cruise at high speed
# 16. temporary stop by roadside
# ---------------------------
def callback_lane_keep(data_decision_basis):
	global lane_keep_basis
	lane_keep_basis = data_decision_basis

def callback_lane_change_to_left(data_decision_basis):
	global lane_change_to_left_basis
	lane_change_to_left_basis = data_decision_basis

def callback_lane_change_to_right(data_decision_basis):
	global lane_change_to_right_basis
	lane_change_to_right_basis = data_decision_basis

def callback_turn_left_at_cross(data_decision_basis):
	global turn_left_at_cross_basis
	turn_left_at_cross_basis = data_decision_basis

def callback_turn_right_at_cross(data_decision_basis):
	global turn_right_at_cross_basis
	turn_right_at_cross_basis = data_decision_basis

def callback_go_straight_at_cross(data_decision_basis):
	global go_straight_at_cross_basis
	go_straight_at_cross_basis = data_decision_basis

def callback_u_turn(data_decision_basis):
	global u_turn_basis
	u_turn_basis = data_decision_basis

def callback_stop_at_obstacle(data_decision_basis):
	global stop_at_obstacle_basis
	stop_at_obstacle_basis = data_decision_basis

def callback_stop_at_red_light(data_decision_basis):
	global stop_at_red_light_basis
	stop_at_red_light_basis = data_decision_basis

def callback_bypass_obstacle_from_left(data_decision_basis):
	global bypass_obstacle_from_left_basis
	bypass_obstacle_from_left_basis = data_decision_basis

def callback_bypass_obstacle_from_right(data_decision_basis):
	global bypass_obstacle_from_right_basis
	bypass_obstacle_from_right_basis = data_decision_basis

def callback_overtake_from_left(data_decision_basis):
	global overtake_from_left_basis
	overtake_from_left_basis = data_decision_basis

def callback_overtake_from_right(data_decision_basis):
	global overtake_from_right_basis
	overtake_from_right_basis = data_decision_basis

def callback_car_follow_in_city(data_decision_basis):
	global car_follow_in_city_basis
	car_follow_in_city_basis = data_decision_basis

def callback_cruise_at_high_speed(data_decision_basis):
	global cruise_at_high_speed_basis
	cruise_at_high_speed_basis = data_decision_basis

def callback_temporary_stop_by_roadside(data_decision_basis):
	global temporary_stop_by_roadside_basis
	temporary_stop_by_roadside_basis = data_decision_basis


def listener():
	rospy.init_node("decision_maker", anonymous=True)
	# subscribers
	rospy.Subscriber("/navigation_command", NaviCommand, callback_navigation, queue_size=1)
	rospy.Subscriber("/decision_basis/surrounding_build", SurroundingBuild, callback_surrounding_build, queue_size=1)
	rospy.Subscriber("/decision_basis/lane_keep", LaneKeepBasis, callback_lane_keep, queue_size=1)
	rospy.Subscriber("/decision_basis/lane_change_to_left", LaneChangeToLeftBasis, callback_lane_change_to_left, queue_size=1)
	rospy.Subscriber("/decision_basis/lane_change_to_right", LaneChangeToRightBasis, callback_lane_change_to_right, queue_size=1)
	rospy.Subscriber("/decision_basis/turn_left_at_cross", TurnLeftAtCrossBasis, callback_turn_left_at_cross, queue_size=1)
	rospy.Subscriber("/decision_basis/turn_right_at_cross", TurnRightAtCrossBasis, callback_turn_right_at_cross, queue_size=1)
	rospy.Subscriber("/decision_basis/go_straight_at_cross", GoStraightAtCrossBasis, callback_go_straight_at_cross, queue_size=1)
	rospy.Subscriber("/decision_basis/u_turn", UTurnBasis, callback_u_turn, queue_size=1)
	rospy.Subscriber("/decision_basis/stop_at_obstacle", StopAtObstacleBasis, callback_stop_at_obstacle, queue_size=1)
	rospy.Subscriber("/decision_basis/stop_at_red_light", StopAtRedLightBasis, callback_stop_at_red_light, queue_size=1)
	rospy.Subscriber("/decision_basis/bypass_obstacle_from_left", BypassObstacleFromLeftBasis, callback_bypass_obstacle_from_left, queue_size=1)
	rospy.Subscriber("/decision_basis/bypass_obstacle_from_right", BypassObstacleFromRightBasis, callback_bypass_obstacle_from_right, queue_size=1)
	rospy.Subscriber("/decision_basis/overtake_from_left", OvertakeFromLeftBasis, callback_overtake_from_left, queue_size=1)
	rospy.Subscriber("/decision_basis/overtake_from_right", OvertakeFromRightBasis, callback_overtake_from_right, queue_size=1)
	rospy.Subscriber("/decision_basis/car_follow_in_city", CarFollowInCityBasis, callback_car_follow_in_city, queue_size=1)
	rospy.Subscriber("/decision_basis/cruise_at_high_speed", CruiseAtHighSpeedBasis, callback_cruise_at_high_speed, queue_size=1)
	rospy.Subscriber("/decision_basis/temporary_stop_by_roadside", TemporaryStopByRoadsideBasis, callback_temporary_stop_by_roadside, queue_size=1)

	# publishers
	pub_steering_strategy = rospy.Publisher("/steering_strategy", XControlStrategy, queue_size=1)
	pub_longitudinal_strategy = rospy.Publisher("/longitudinal_strategy", YControlStrategy, queue_size=1)

	rate = rospy.Rate(10)

	# navigation info
	global navi_command
	navi_command = NaviCommand()
	# surrounding build
	global surrounding
	surrounding = SurroundingBuild()
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
	



