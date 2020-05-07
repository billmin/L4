import os, sys
from enum import Enum
import random

class DrvStyle(Enum):
	MILD    = 0
	NORMAL  = 1
	RADICAL = 2

def determined_to_execute(driving_style=DrvStyle.NORMAL):
	p = random.random()
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


