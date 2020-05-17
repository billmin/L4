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
from multi_task_msgs import SurroundingBuild
from decision_basis_msgs import Decision
from complementary import NaviCommand

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
def callback_lane_keep(data_decision):
	global lane_keep_decision
	lane_keep_basis = data_decision

def callback_lane_change_to_left(data_decision):
	global lane_change_to_left_decision
	lane_change_to_left_basis = data_decision

def callback_lane_change_to_right(data_decision):
	global lane_change_to_right_decision
	lane_change_to_right_basis = data_decision

def callback_turn_left_at_cross(data_decision):
	global turn_left_at_cross_decision
	turn_left_at_cross_basis = data_decision

def callback_turn_right_at_cross(data_decision):
	global turn_right_at_cross_decision
	turn_right_at_cross_basis = data_decision

def callback_go_straight_at_cross(data_decision):
	global go_straight_at_cross_decision
	go_straight_at_cross_basis = data_decision

def callback_u_turn(data_decision):
	global u_turn_decision
	u_turn_basis = data_decision

def callback_stop_at_obstacle(data_decision):
	global stop_at_obstacle_decision
	stop_at_obstacle_basis = data_decision

def callback_stop_at_red_light(data_decision):
	global stop_at_red_light_decision
	stop_at_red_light_basis = data_decision

def callback_bypass_obstacle_from_left(data_decision):
	global bypass_obstacle_from_left_decision
	bypass_obstacle_from_left_basis = data_decision

def callback_bypass_obstacle_from_right(data_decision):
	global bypass_obstacle_from_right_decision
	bypass_obstacle_from_right_basis = data_decision

def callback_overtake_from_left(data_decision):
	global overtake_from_left_decision
	overtake_from_left_basis = data_decision

def callback_overtake_from_right(data_decision):
	global overtake_from_right_decision
	overtake_from_right_basis = data_decision

def callback_car_follow_in_city(data_decision):
	global car_follow_in_city_decision
	car_follow_in_city_basis = data_decision

def callback_cruise_at_high_speed(data_decision):
	global cruise_at_high_speed_decision
	cruise_at_high_speed_basis = data_decision

def callback_temporary_stop_by_roadside(data_decision):
	global temporary_stop_by_roadside_decision
	temporary_stop_by_roadside_basis = data_decision


def listener():
	rospy.init_node("decision_maker", anonymous=True)
	# subscribers
	rospy.Subscriber("/navigation_command", NaviCommand, callback_navigation, queue_size=1)
	rospy.Subscriber("/multi_task/surrounding_build", SurroundingBuild, callback_surrounding_build, queue_size=1)
	rospy.Subscriber("/lane_keep/decision", Decision, callback_lane_keep, queue_size=1)
	rospy.Subscriber("/lane_change_to_left/decision", Decision, callback_lane_change_to_left, queue_size=1)
	rospy.Subscriber("/lane_change_to_right/decision", Decision, callback_lane_change_to_right, queue_size=1)
	rospy.Subscriber("/turn_left_at_cross/decision", Decision, callback_turn_left_at_cross, queue_size=1)
	rospy.Subscriber("/turn_right_at_cross/decision", Decision, callback_turn_right_at_cross, queue_size=1)
	rospy.Subscriber("/go_straight_at_cross/decision", Decision, callback_go_straight_at_cross, queue_size=1)
	rospy.Subscriber("/u_turn/decision", Decision, callback_u_turn, queue_size=1)
	rospy.Subscriber("/stop_at_obstacle/decision", Decision, callback_stop_at_obstacle, queue_size=1)
	rospy.Subscriber("/stop_at_red_light/decision", Decision, callback_stop_at_red_light, queue_size=1)
	rospy.Subscriber("/bypass_obstacle_from_left/decision", Decision, callback_bypass_obstacle_from_left, queue_size=1)
	rospy.Subscriber("/bypass_obstacle_from_right/decision", Decision, callback_bypass_obstacle_from_right, queue_size=1)
	rospy.Subscriber("/overtake_from_left/decision", Decision, callback_overtake_from_left, queue_size=1)
	rospy.Subscriber("/overtake_from_right/decision", Decision, callback_overtake_from_right, queue_size=1)
	rospy.Subscriber("/car_follow_in_city/decision", Decision, callback_car_follow_in_city, queue_size=1)
	rospy.Subscriber("/cruise_at_high_speed/decision", Decision, callback_cruise_at_high_speed, queue_size=1)
	rospy.Subscriber("/temporary_stop_by_roadside/decision", Decision, callback_temporary_stop_by_roadside, queue_size=1)

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

	# decision for lane keep
	global lane_keep_decision
	lane_keep_decision = Decision()

	# decision for lane change to left
	global lane_change_to_left_decision
	lane_change_to_left_decision = Decision()

	# decision for lane change to right
	global lane_change_to_right_decision
	lane_change_to_right_decision = Decision()

	# decision for turning left at cross
	global turn_left_at_cross_decision
	turn_left_at_cross_decision = Decision()

	# decision for turning right at cross
	global turn_right_at_cross_decision
	turn_right_at_cross_decision = Decision()

	# decision for going straight at cross
	global go_straight_at_cross_decision
	go_straight_at_cross_decision = Decision()

	# decision for u turn
	global u_turn_decision
	u_turn_decision = Decision()

	# decision for stopping at obstacle
	global stop_at_obstacle_decision
	stop_at_obstacle_decision = Decision()

	# decision for stopping at red light
	global stop_at_red_light_decision
	stop_at_red_light_decision = Decision()

	# decision for bypassing obstacle from left
	global bypass_obstacle_from_left_decision
	bypass_obstacle_from_left_decision = Decision()

	# decision for bypassing obstacle from right
	global bypass_obstacle_from_right_decision
	bypass_obstacle_from_right_decision = Decision()

	# decision for overtaking from left
	global overtake_from_left_decision
	overtake_from_left_decision = Decision()

	# decision for overtaking from right
	global overtake_from_right_decision
	overtake_from_right_decision = Decision()

	# decision for car following in city
	global car_follow_in_city_decision
	car_follow_in_city_decision = Decision()

	# decision for cruise at high speed
	global cruise_at_high_speed_decision
	cruise_at_high_speed_decision = Decision()

	# decision for temporary stop by the roadside
	global temporary_stop_by_roadside_decision
	temporary_stop_by_roadside_decision = Decision()

	# navigation based
	while not rospy.is_shutdown():
		navi_instruction = navi_command.navi_instruction
		if navi_instruction == NaviCommand.GO_STRAIGHT.value:
			
		elif navi_instruction == NaviCommand.TURN_LEFT_AT_CROSS.value:
			
		elif navi_instruction == NaviCommand.TURN_RIGHT_AT_CROSS.value:

		elif navi_instruction == NaviCommand.AHEAD_LEFT_AT_FORK.value:

		elif navi_instruction == NaviCommand.AHEAD_RIGHT_AT_FORK.value:


		elif navi_instruction == NaviCommand.TURN_LEFT_TOWARDS_REAR.value:


		elif navi_instruction == NaviCommand.TURN_RIGHT_TOWARDS_REAR.value:


		elif navi_instruction == NaviCommand.U_TURN.value:


		elif navi_instruction == NaviCommand.TUNNEL.value:


		elif navi_instruction == NaviCommand.ARRIVED.value:
		

		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



