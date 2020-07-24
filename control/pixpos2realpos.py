import time
import os, sys
import numpy as np
from random import randint


def _quicksort(array):
	if len(array) < 2:
		return array

	low, same, high = [], [], []

	pivot = array[randint(0, len(array)-1)]

	for item in array:
		if item < pivot:
			low.append(item)
		elif item == pivot:
			same.append(item)
		else:
			high.append(item)

	return _quicksort(low) + same + _quicksort(high)
	
				
def estimate_real_location(current_pixel_x, current_pixel_y, ref_pixel2realcoords, total_closest=10):
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
	mark_dist_to_ref_pts = _quicksort(mark_dist_to_ref_pts)
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
	
