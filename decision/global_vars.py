from enum import Enum

## ----------- ##
# mild: change lane when there are no other vehicles within 50 meters
# normal: change lane when other neighbor vehicles are 20 meters away and the speed can guarantee self-vehicle to finish lane change before collision
# radical: change lane when other neighbor vehicles are 5 meters away and the speed can guarantee self-vehicle to finishi lane change before collision, otherwise, self-vehicle shall wait or tentatively change lane and abort that if warned by others
## ----------- ##
class DrvStyle(Enum):
	MILD    = 0
	NORMAL  = 1
	RADICAL = 2

class Riskness(Enum):
	PRETTY_SAFE = 0
	MEDIUM_RISK = 1
	HIGH_RISK = 2

class PATIENCE(Enum):
	PATIENT   = 0
	IMPATIENT = 1

# navigation commands
class NaviCommand(Enum):
	GO_STRAIGHT = 0
	TURN_LEFT = 1
	TURN_RIGHT = 2
	TURN_SLIGHT_LEFT = 3
	TURN_SLIGHT_RIGHT = 4
	TURN_SHARP_LEFT = 5
	TURN_SHARP_RIGHT = 6
	U_TURN = 7
	DEPART = 8
	ARRIVED = 9
	MERGE_ON_LEFT = 10
	MERGE_ON_RIGHT = 11
	ON_RAMP = 12
	OFF_RAMP = 13
	FORK_LEFT = 14
	FORK_RIGHT = 15
	END_OF_ROAD_LEFT = 16
	END_OF_ROAD_RIGHT = 17
	END_OF_ROAD_FRONT = 18
	CONTINUE = 19
	ENTER_ROUNDABOUT_ON_LEFT = 20
	ENTER_ROUNDABOUT_ON_RIGHT = 21
	EXIT_ROUNDABOUT = 22
	
	
