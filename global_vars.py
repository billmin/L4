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

# navigation
class NaviCommand(Enum):
	GO_STRAIGHT = 0
	TURN_LEFT_AT_CROSS = 1
	TURN_RIGHT_AT_CROSS = 2
	AHEAD_LEFT_AT_FORK = 3
	AHEAD_RIGHT_AT_FORK = 4
	TURN_LEFT_TOWARDS_REAR = 5
	TURN_RIGHT_TOWARDS_REAR = 6
	U_TURN = 7
	TUNNEL = 8
	ARRIVED = 9
