from transitions import Machine

class Navigation:
	pass
		
# navigation related states
navi_states = {0:  'PullOver',
			   1:  'GoStraightAlongRoad',
			   2:  'TurnLeftAtCrossing',
			   3:  'TurnRightAtCrossing',
			   4:  'TurnSlightLeftAhead',
			   5:  'TurnSlightRightAhead',
			   6:  'TurnSharpLeftAhead',
			   7:  'TurnSharpRightAhead',
			   8:  'UTurn',
			   9:  'MergeIntoMainStreamOnLeft',
			   10: 'MergeIntoMainStreamOnRight',
			   11: 'OnRampIntoHighway',
			   12: 'OffRampOutOfHighway',
			   13: 'GoLeftAtForkAhead',
			   14: 'GoRightAtForkAhead',
			   15: 'EnterRoundaboutOnLeft',
			   16: 'EnterRoundaboutOnRight',
			   17: 'ExitRoundabout'}

# navigation related warnings
navi_warnings = {0: 'end_of_road_left',
				 1: 'end_of_road_right',
				 2: 'end_of_road_front'} 

# navi command
navi_commands = {0:  'depart',
				 1:  'arrived',
				 2:  'go_straight',
				 3:  'turn_left',
				 4:  'turn_right',
				 5:  'turn_slight_left',
				 6:  'turn_slight_right',
				 7:  'turn_sharp_left',
				 8:  'turn_sharp_right',
				 9:  'u_turn',
				 10: 'merge_on_left',
				 11: 'merge_on_right',
				 12: 'on_ramp',
				 13: 'off_ramp',
				 14: 'fork_left',
				 15: 'fork_right',
				 16: 'continue',
				 17: 'enter_roundabout_on_left',
				 18: 'enter_roundabout_on_right',
				 19: 'exit_roundabout'}

# initial state shall be pullover!
# when arriving at dest, run this
mission_completed = ['pull_over']

# transition of maneuver-related tasks in a redundant way
transitions = [
	# current state: pull over [0]
	{'trigger': navi_commands[0], 'source': navi_states[0], 'dest': navi_states[1]},
	# current state: go straight along road [1]
	{'trigger': navi_commands[1], 'source': navi_states[1], 'dest': navi_states[0]},
	{'trigger': navi_commands[2], 'source': navi_states[1], 'dest': navi_states[1]},
	{'trigger': navi_commands[16], 'source': navi_states[1], 'dest': navi_states[1]},
	{'trigger': navi_commands[3], 'source': navi_states[1], 'dest': navi_states[2]},
	{'trigger': navi_commands[4], 'source': navi_states[1], 'dest': navi_states[3]},
	{'trigger': navi_commands[5], 'source': navi_states[1], 'dest': navi_states[4]},
	{'trigger': navi_commands[6], 'source': navi_states[1], 'dest': navi_states[5]},
	{'trigger': navi_commands[7], 'source': navi_states[1], 'dest': navi_states[6]},
	{'trigger': navi_commands[8], 'source': navi_states[1], 'dest': navi_states[7]},
	{'trigger': navi_commands[9], 'source': navi_states[1], 'dest': navi_states[8]},
	{'trigger': navi_commands[10], 'source': navi_states[1], 'dest': navi_states[9]},
	{'trigger': navi_commands[11], 'source': navi_states[1], 'dest': navi_states[10]},
	{'trigger': navi_commands[12], 'source': navi_states[1], 'dest': navi_states[11]},
	{'trigger': navi_commands[13], 'source': navi_states[1], 'dest': navi_states[12]},
	{'trigger': navi_commands[14], 'source': navi_states[1], 'dest': navi_states[13]},
	{'trigger': navi_commands[15], 'source': navi_states[1], 'dest': navi_states[14]},
	{'trigger': navi_commands[17], 'source': navi_states[1], 'dest': navi_states[15]},
	{'trigger': navi_commands[18], 'source': navi_states[1], 'dest': navi_states[16]},
	{'trigger': navi_commands[19], 'source': navi_states[1], 'dest': navi_states[17]},
	# current state: turn left at crossing [2]
	{'trigger': navi_commands[2], 'source': navi_states[2], 'dest': navi_states[1]},
	{'trigger': navi_commands[3], 'source': navi_states[2], 'dest': navi_states[2]},
	{'trigger': navi_commands[16], 'source': navi_states[2], 'dest': navi_states[2]},
	# current state: turn right at crossing [3]
	# current state: turn slight left ahead [4]
	# current state: turn slight right ahead [5]
	# current state: turn sharp left ahead [6]
	# current state: turn sharp right ahead [7]
	# current state: u turn	[8]
	# current state: merge into main stream on left [9]
	# current state: merge into main stream on right [10]
	# current state: on ramp into highway [11]
	# current state: off ramp out of highway [12]
	# current state: go left at fork ahead [13]
	# current state: go right at fork ahead [14]
	# current state: continue with current road [15]
	# current state: enter roundabout on left [16]
	# current state: enter roundabout on right [17]
	# current state: exit roundabout [18]
	]
	
