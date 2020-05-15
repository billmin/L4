import os, sys
from enum import Enum
from complementary import DrvStyle, PATIENCE
import random


def is_patient_for_hindering(patience=PATIENCE.PATIENT.value, urgent=False):
	low = 0.3	
	high = 0.8
	# roll the dice
	p = random.random()
	
	if not urgent:
		if patience == PATIENCE.PATIENT.value:
			if 0.0 <= p <= high:
				return True
			else:
				return False
		else:
			if 0.0 <= p <= low:
				return True
			else:
				return False
	else:
		return False


def is_determined_to_bear_risk(driving_style=DrvStyle.NORMAL.value, urgent=False):
	low = 0.2
	medium = 0.5
	high = 0.8
	# roll the dice
	p = random.random()
	if not urgent:
		if style == DrvStyle.NORMAL.value:
			if 0.0 <= p <= medium:
				return True
			else:
				return False
		elif style == DrvStyle.MILD.value:
			if 0.0 <= p	<= low:
				return True
			else:
				return False
		else:
			if 0.0 <= p <= high:
				return True
			else:
				return False
	else:
		return True



