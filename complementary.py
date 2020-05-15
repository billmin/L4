from enum import Enum


class DrvStyle(Enum):
	MILD    = 0
	NORMAL  = 1
	RADICAL = 2

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
