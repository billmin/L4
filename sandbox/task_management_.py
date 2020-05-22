# navigation commands
navigation_commands = ['nc_set_off',
					   'nc_go_straight_along_the_road',
					   'nc_go_straight_at_cross',
					   'nc_turn_left_at_cross',
					   'nc_turn_right_at_cross',
					   'nc_choose_leftmost_branch_at_fork',
					   'nc_choose_rightmost_branch_at_fork',
					   'nc_choose_middle_branch_at_fork',
					   'nc_afflux_into_flow',
					   'nc_drive_into_ramp',
					   'nc_turn_left_towards_rear',
					   'nc_turn_right_towards_rear',
					   'nc_u_turn',
					   'nc_arrived',]


# navigation-lbs(to be continued)
navigation_lbs = ['tunnel',
				  'service_area',
				  'gasoline',]


# maneuver-level tasks
tasks = {0: 'wide_speed_range_adaptive_cruise',
		 1: 'lane_change_to_left',
		 2: 'lane_change_to_right',
		 3: 'turn_left_at_cross',
		 4: 'turn_right_at_cross',
		 5: 'go_straight_at_cross',
		 6: 'u_turn',
		 7: 'stop_at_obstacle',
		 8: 'halt_behind_stop_line',
		 9: 'slow_down_for_pedestrian_traverse',
		 10: 'bypass_obstacle_on_left',
		 11: 'bypass_obstacle_on_right',
		 12: 'overtake_on_left',
		 13: 'overtake_on_right',
		 14: 'temporary_stop_by_roadside',
		 15: 'emergency_return_to_initial_lane'}

# trigger definition
triggers = {0: 'request_wide_speed_range_adaptive_cruise',
			1: 'request_lane_change_to_left',
		 	2: 'request_lane_change_to_right',
		 	3: 'request_turn_left_at_cross',
		 	4: 'request_turn_right_at_cross',
		 	5: 'request_go_straight_at_cross',
		 	6: 'request_u_turn',
		 	7: 'request_stop_at_obstacle',
		 	8: 'request_halt_behind_stop_line',
		 	9: 'request_slow_down_for_pedestrian_traverse',
		 	10: 'request_bypass_obstacle_on_left',
		 	11: 'request_bypass_obstacle_on_right',
		 	12: 'request_overtake_on_left',
		 	13: 'request_overtake_on_right',
		 	14: 'request_temporary_stop_by_roadside',
			15: 'request_mission_abort'}

# transition of maneuver-related tasks
transitions = [
	# current task: wide_speed_range_adaptive_cruise [0]
	{'trigger': triggers[0], 'source': tasks[0], 'dest': tasks[0]},
	{'trigger': triggers[1], 'source': tasks[0], 'dest': tasks[1]},
	{'trigger': triggers[2], 'source': tasks[0], 'dest': tasks[2]},
	{'trigger': triggers[3], 'source': tasks[0], 'dest': tasks[3]},
	{'trigger': triggers[4], 'source': tasks[0], 'dest': tasks[4]},
	{'trigger': triggers[5], 'source': tasks[0], 'dest': tasks[5]},
	{'trigger': triggers[6], 'source': tasks[0], 'dest': tasks[6]},
	{'trigger': triggers[7], 'source': tasks[0], 'dest': tasks[7]},
	{'trigger': triggers[8], 'source': tasks[0], 'dest': tasks[8]},
	{'trigger': triggers[9], 'source': tasks[0], 'dest': tasks[9]},
	{'trigger': triggers[10], 'source': tasks[0], 'dest': tasks[10]},
	{'trigger': triggers[11], 'source': tasks[0], 'dest': tasks[11]},
	{'trigger': triggers[12], 'source': tasks[0], 'dest': tasks[12]},
	{'trigger': triggers[13], 'source': tasks[0], 'dest': tasks[13]},
	{'trigger': triggers[14], 'source': tasks[0], 'dest': tasks[14]},
	# current task: lane_change_to_left [1]
	{'trigger': triggers[0], 'source': tasks[1], 'dest': tasks[0]},
	{'trigger': triggers[1], 'source': tasks[1], 'dest': tasks[1]},
	{'trigger': triggers[15], 'source': tasks[1], 'dest': tasks[15]},
	# current task: lane_change_to_right [2]
	{'trigger': triggers[0], 'source': tasks[2], 'dest': tasks[0]},
	{'trigger': triggers[2], 'source': tasks[2], 'dest': tasks[2]},
	{'trigger': triggers[15], 'source': tasks[2], 'dest': tasks[15]},
	# current task: turn_left_at_cross [3]
	{'trigger': triggers[0], 'source': tasks[3], 'dest': tasks[0]},
	{'trigger': triggers[3], 'source': tasks[3], 'dest': tasks[3]},
	{'trigger': triggers[10], 'source': tasks[3], 'dest': tasks[10]},
	{'trigger': triggers[11], 'source': tasks[3], 'dest': tasks[11]},
	# current task:  turn_right_at_cross [4]
	{'trigger': triggers[0], 'source': tasks[4], 'dest': tasks[0]},
	{'trigger': triggers[4], 'source': tasks[4], 'dest': tasks[4]},
	{'trigger': triggers[10], 'source': tasks[4], 'dest': tasks[10]},
	{'trigger': triggers[11], 'source': tasks[4], 'dest': tasks[11]},
	# current task: go_straight_at_cross [5]
	{'trigger': triggers[0], 'source': tasks[5], 'dest': tasks[0]},
	{'trigger': triggers[5], 'source': tasks[5], 'dest': tasks[5]},
	{'trigger': triggers[10], 'source': tasks[5], 'dest': tasks[10]},
	{'trigger': triggers[11], 'source': tasks[5], 'dest': tasks[11]},
	{'trigger': triggers[12], 'source': tasks[5], 'dest': tasks[12]},
	{'trigger': triggers[13], 'source': tasks[5], 'dest': tasks[13]},
	# current task: u_turn [6]
	{'trigger': triggers[0], 'source': tasks[6], 'dest': tasks[0]},
	{'trigger': triggers[6], 'source': tasks[6], 'dest': tasks[6]},
	{'trigger': triggers[10], 'source': tasks[6], 'dest': tasks[10]},
	{'trigger': triggers[11], 'source': tasks[6], 'dest': tasks[11]},
	# current task: stop_at_obstacle [7]
	{'trigger': triggers[0], 'source': tasks[7], 'dest': tasks[0]},
	{'trigger': triggers[3], 'source': tasks[7], 'dest': tasks[3]},
	{'trigger': triggers[4], 'source': tasks[7], 'dest': tasks[4]},
	{'trigger': triggers[5], 'source': tasks[7], 'dest': tasks[5]},
	{'trigger': triggers[7], 'source': tasks[7], 'dest': tasks[7]},
	{'trigger': triggers[10], 'source': tasks[7], 'dest': tasks[10]},
	{'trigger': triggers[11], 'source': tasks[7], 'dest': tasks[11]},
	# current task: halt_behind_stop_line [8]
	{'trigger': triggers[0], 'source': tasks[8], 'dest': tasks[0]},
	{'trigger': triggers[3], 'source': tasks[8], 'dest': tasks[3]},
	{'trigger': triggers[4], 'source': tasks[8], 'dest': tasks[4]},
	{'trigger': triggers[5], 'source': tasks[8], 'dest': tasks[5]},
	{'trigger': triggers[8], 'source': tasks[8], 'dest': tasks[8]},
	{'trigger': triggers[10], 'source': tasks[8], 'dest': tasks[10]},
	{'trigger': triggers[11], 'source': tasks[8], 'dest': tasks[11]},
	# current task: slow_down_for_pedestrian_traverse [9]	
	{'trigger': triggers[0], 'source': tasks[9], 'dest': tasks[0]},
	{'trigger': triggers[1], 'source': tasks[9], 'dest': tasks[1]},
	{'trigger': triggers[2], 'source': tasks[9], 'dest': tasks[2]},
	{'trigger': triggers[3], 'source': tasks[9], 'dest': tasks[3]},
	{'trigger': triggers[4], 'source': tasks[9], 'dest': tasks[4]},
	{'trigger': triggers[5], 'source': tasks[9], 'dest': tasks[5]},
	{'trigger': triggers[8], 'source': tasks[9], 'dest': tasks[8]},
	{'trigger': triggers[9], 'source': tasks[9], 'dest': tasks[9]},
	{'trigger': triggers[10], 'source': tasks[9], 'dest': tasks[10]},
	{'trigger': triggers[11], 'source': tasks[9], 'dest': tasks[11]},
	# current task: bypass_obstacle_on_left [10]
	{'trigger': triggers[0], 'source': tasks[10], 'dest': tasks[0]},
	{'trigger': triggers[1], 'source': tasks[10], 'dest': tasks[1]},
	{'trigger': triggers[2], 'source': tasks[10], 'dest': tasks[2]},
	{'trigger': triggers[3], 'source': tasks[10], 'dest': tasks[3]},
	{'trigger': triggers[4], 'source': tasks[10], 'dest': tasks[4]},
	{'trigger': triggers[5], 'source': tasks[10], 'dest': tasks[5]},
	{'trigger': triggers[6], 'source': tasks[10], 'dest': tasks[6]},
	{'trigger': triggers[7], 'source': tasks[10], 'dest': tasks[7]},
	{'trigger': triggers[8], 'source': tasks[10], 'dest': tasks[8]},
	{'trigger': triggers[9], 'source': tasks[10], 'dest': tasks[9]},
	{'trigger': triggers[10], 'source': tasks[10], 'dest': tasks[10]},
	{'trigger': triggers[11], 'source': tasks[10], 'dest': tasks[11]},
	{'trigger': triggers[12], 'source': tasks[10], 'dest': tasks[12]},
	{'trigger': triggers[13], 'source': tasks[10], 'dest': tasks[13]},
	{'trigger': triggers[14], 'source': tasks[10], 'dest': tasks[14]},
	{'trigger': triggers[15], 'source': tasks[10], 'dest': tasks[15]},

	# current task: bypass_obstacle_on_right [11]
	{'trigger': triggers[0], 'source': tasks[11], 'dest': tasks[0]},
	{'trigger': triggers[1], 'source': tasks[11], 'dest': tasks[1]},
	{'trigger': triggers[2], 'source': tasks[11], 'dest': tasks[2]},
	{'trigger': triggers[3], 'source': tasks[11], 'dest': tasks[3]},
	{'trigger': triggers[4], 'source': tasks[11], 'dest': tasks[4]},
	{'trigger': triggers[5], 'source': tasks[11], 'dest': tasks[5]},
	{'trigger': triggers[6], 'source': tasks[11], 'dest': tasks[6]},
	{'trigger': triggers[7], 'source': tasks[11], 'dest': tasks[7]},
	{'trigger': triggers[8], 'source': tasks[11], 'dest': tasks[8]},
	{'trigger': triggers[9], 'source': tasks[11], 'dest': tasks[9]},
	{'trigger': triggers[10], 'source': tasks[11], 'dest': tasks[10]},
	{'trigger': triggers[11], 'source': tasks[11], 'dest': tasks[11]},
	{'trigger': triggers[12], 'source': tasks[11], 'dest': tasks[12]},
	{'trigger': triggers[13], 'source': tasks[11], 'dest': tasks[13]},
	{'trigger': triggers[14], 'source': tasks[11], 'dest': tasks[14]},
	{'trigger': triggers[15], 'source': tasks[11], 'dest': tasks[15]},

	# current task: overtake_on_left [12]
	{'trigger': triggers[0], 'source': tasks[9], 'dest': tasks[0]},

	# current task: overtake_on_right [13]
	{'trigger': triggers[0], 'source': tasks[9], 'dest': tasks[0]},

	# current task: temporary_stop_by_roadside [14]
	{'trigger': triggers[0], 'source': tasks[9], 'dest': tasks[0]},

	# current task: emergency_return_to_initial_lane [15]
	{'trigger': triggers[0], 'source': tasks[9], 'dest': tasks[0]},
	




	{'trigger': triggers[0], 'source': tasks[2], 'dest': tasks[0]}]
	
