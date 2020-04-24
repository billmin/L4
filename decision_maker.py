import os, sys


class XAction:
	turn_left = 0
	go_straight = 1
	turn_right = 2
	
class YAction:
	constant = 0
	speed_up = 1
	speed_down = 2

class Direction:
	forward = 0
	backward = 1

class FPVOrientation:
	'''
		o
	x	|	x
		|
	o---+---o
		|
	x	|	x
		o
	'''
	ahead = 0
	rear = 1
	left = 2
	right = 3
	left_ahead = 4
	left_rear = 5
	right_ahead = 6
	right_rear = 7

class Velocity:
	stop = 0
	moving_forward = 1
	moving_backward = 2


def action_encoder(x_action, y_action, y_direction):
	return x_action | (y_action<<2) | (y_direction<<4)

	
def action_decoder(encoded_action):
	x_mask = 0b11
	y_mask = 0b1100
	direct_mask = 0b10000
	x_action = encoded_action & x_mask
	y_action = encoded_action & y_mask
	y_direction = encoded_action & direct_mask
	return x_action, y_action, y_direction

def coarse_action(fpvo, vel):
	if fpvo == FPVOrientation.ahead:
		if vel == Velocity.stop:
			return action_encoder(XAction.go_straight, YAction.speed_up, Direction.forward)
		elif vel == Velocity.moving_forward:
			return action_encoder(XAction.go_straight, YAction.constant, Direction.forward)
		else:
			return action_encoder(XAction.go_straight, YAction.speed_down, Direction.backward)

	elif fpvo == FPVOrientation.rear:
		if vel == Velocity.stop:
			return action_encoder(XAction.go_straight, YAction.speed_up, Direction.backward)
		elif vel == Velocity.moving_forward:
			return action_encoder(XAction.go_straight, YAction.speed_down, Direction.forward)
		else:
			return action_encoder(XAction.go_straight, YAction.constant, Direction.backward)

	elif fpvo == FPVOrientation.left:
		if vel == Velocity.stop:
			return action_encoder(XAction.turn_left, YAction.speed_up, Direction.forward)
		elif vel == Velocity.moving_forward:
			return action_encoder(XAction.turn_left, YAction.constant, Direction.forward)
		else:
			return action_encoder(XAction.turn_left, YAction.speed_down, Direction.backward)
			
	elif fpvo == FPVOrientation.right:
		if vel == Velocity.stop:
			return action_encoder(XAction.turn_right, YAction.speed_up, Direction.forward)
		elif vel == Velocity.moving_forward:
			return action_encoder(XAction.turn_right, YAction.constant, Direction.forward)
		else:
			return action_encoder(XAction.turn_right, YAction.speed_down, Direction.backward)
			
	elif fpvo == FPVOrientation.left_ahead:

	elif fpvo == FPVOrientation.left_rear:

	elif fpvo == FPVOrientation.right_ahead:

	else:
		



