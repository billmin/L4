import os, sys
import numpy as np
import json


#class LaneDetectionProcess:
#	def __init__(self, detected_line_coords, ):
		



with open('coords_x_axis_rr.json', 'r') as file:
	# get all pxiel2real coordinate pairs
	coords_x_axis = json.load(file)

	for k, v in coords_x_axis.items():
		print(k, v)
		
