import cv2
import time
import os, sys
import numpy as np
import json
from quick_sort import quicksort

# global vars
width = 640
height = 360
nuget_size = 0.12

def _bubble_sort(data_list, order_down=False):
	data_len = len(data_list)
	for i in range(data_len-1):
		for j in range(data_len-1-i):
			if order_down:
				if(data_list[j][0] < data_list[j+1][0]):
					data_list[j], data_list[j+1] = data_list[j+1], data_list[j]
			else:
				if(data_list[j][0] > data_list[j+1][0]):
					data_list[j], data_list[j+1] = data_list[j+1], data_list[j]
					
				
def getRealLocation(current_pixel_x, current_pixel_y, ref_pixel2realcoords, total_closest=20):
	mark_dist_to_ref_pts = []
	##
	t0 = time.time()
	for k_pix2real, v_pix2real in ref_pixel2realcoords.items():
		description_dist_to_ref_pt = (current_pixel_x - v_pix2real[0])**2 + (current_pixel_y - v_pix2real[1])**2
		mark_dist_to_ref_pts.append((description_dist_to_ref_pt, k_pix2real))
	t1 = time.time()
	print(t1-t0)
	t2 = time.time()
	# sort
	#_bubble_sort(mark_dist_to_ref_pts)
	mark_dist_to_ref_pts = quicksort(mark_dist_to_ref_pts)
	t3 = time.time()
	print(t3-t2)
	# get marked info of closest ref points
	mark_dist_to_closest_ref_pts = mark_dist_to_ref_pts[0:total_closest]
	# extract distance info of current pos to closest ref points
	dist_info = [np.sqrt(mark[0]) for mark in mark_dist_to_closest_ref_pts]
	weight_dist = [np.exp(-dist) for dist in dist_info]
	# normalization
	normalized_weight = [w/np.sum(weight_dist) for w in weight_dist]
	
	# estimated real x and real y of current pos
	current_real_x, current_real_y = 0.0, 0.0
	for i, mark in enumerate(mark_dist_to_closest_ref_pts):
		k = mark[1]
		current_real_x += ref_pixel2realcoords[k][2]*normalized_weight[i]
		current_real_y += ref_pixel2realcoords[k][3]*normalized_weight[i]
	
	return current_real_x, current_real_y
	

print("Started...")
with open('pix2real.json', 'r') as file:
	# get all pxiel2real coordinate pairs
	pixel2realcoords = json.load(file)	
	'''
	for k, v_pix2real in pixel2realcoords.items():
		dist_to_origin = np.sqrt(v_pix2real[0]**2 + v_pix2real[1]**2)
	'''
	# test
	current_pixel_x = 400
	current_pixel_y = 256
	current_real_x, current_real_y = getRealLocation(current_pixel_x/width, current_pixel_y/height, pixel2realcoords)
	print("x={}, y={}".format(current_real_x, current_real_y))
