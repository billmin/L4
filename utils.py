import os, sys
from enum import Enum
import random

class DrvStyle(Enum):
	MILD    = 0
	NORMAL  = 1
	RADICAL = 2

def patience_for_hindering():

def determined_to_bear_risk(driving_style=DrvStyle.NORMAL, urgent=False):
	p = random.random()
	if not urgent:
		if style == DrvStyle.NORMAL:
			if 0.0 <= p <= 0.5:
				return True
			else:
				return False
		elif style == DrvStyle.MILD:
			if 0.0 <= p	<= 0.2:
				return True
			else:
				return False
		else:
			if 0.0 <= p <= 0.8:
				return True
			else:
				return False
	else:
		return True



