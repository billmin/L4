import os, sys
import numpy as np
import cv2
import time
import copy
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int8
from cqpilot_msg.msg import mmt_line
from cqpilot_msg.msg import lines
from lane_detection_process_pipeline import LaneDetectionProcessPipeline
from pose_estimator import PoseEstimator
from configparser import ConfigParser
import json

'''
std_msgs/Header header
uint8      line_number
mmt_line[] all_lines


int8                id
int8              	point_num
float32             score                
float32[72]         positionx
float32[72]         positiony
int8                xushi
'''


def callback_detected_left_front_lane_lines(detected_lines):
	global all_detected_lines_lf
	all_detected_lines_lf = detected_lines

	
def callback_detected_right_front_lane_lines(detected_lines):
	global all_detected_lines_rf
	all_detected_lines_rf = detected_lines


def callback_detected_left_rear_lane_lines(detected_lines):
	global all_detected_lines_lr
	all_detected_lines_lr = detected_lines


def callback_detected_right_rear_lane_lines(detected_lines):
	global all_detected_lines_rr
	all_detected_lines_rr = detected_lines


def listener():
	rospy.init_node("show_vehicle_pose_in_lane", anonymous=True)
	
	rospy.Subscriber("/detection/lines/deadzone_lf", lines, callback_detected_left_front_lane_lines, queue_size=1)
	rospy.Subscriber("/detection/lines/deadzone_rf", lines, callback_detected_right_front_lane_lines, queue_size=1)
	rospy.Subscriber("/detection/lines/deadzone_lr", lines, callback_detected_left_rear_lane_lines, queue_size=1)
	rospy.Subscriber("/detection/lines/deadzone_rr", lines, callback_detected_right_rear_lane_lines, queue_size=1)

	rate = rospy.Rate(20)

	# left front
	global all_detected_lines_lf
	all_detected_lines_lf = lines()
	# right front
	global all_detected_lines_rf
	all_detected_lines_rf = lines()
	# left rear
	global all_detected_lines_lr
	all_detected_lines_lr = lines()
	# right rear
	global all_detected_lines_rr
	all_detected_lines_rr = lines()

	# left front ref 
	with open('./dat/lf_ref.json', 'r') as file:
		left_front_ref = json.load(file)
	# right front ref
	with open('./dat/rf_ref.json', 'r') as file:
		right_front_ref = json.load(file)
	# left rear ref
	with open('./dat/lr_ref.json', 'r') as file:
		left_rear_ref = json.load(file)
	# right rear ref
	with open('./dat/rr_ref.json', 'r') as file:
		right_rear_ref = json.load(file)

	# left front pix2real map
	with open('./dat/pix2real_lf.json', 'r') as file:
		left_front_pix_real_pos_map = json.load(file)
	# right front pix2real map
	with open('./dat/pix2real_rf.json', 'r') as file:
		right_front_pix_real_pos_map = json.load(file)
	# left rear pix2real map
	with open('./dat/pix2real_lr.json', 'r') as file:
		left_rear_pix_real_pos_map = json.load(file)
	# right rear pix2real map
	with open('./dat/pix2real_rr.json', 'r') as file:
		right_rear_pix_real_pos_map = json.load(file)
	

	# lane process
	ldpp = LaneDetectionProcessPipeline(left_front_pix_real_pos_map,
										right_front_pix_real_pos_map,
										left_rear_pix_real_pos_map,
										right_rear_pix_real_pos_map,
										left_front_ref,
										right_front_ref,
										left_rear_ref,
										right_rear_ref,
										all_detected_lines_lf,
										all_detected_lines_rf,
										all_detected_lines_lr,
										all_detected_lines_rr)

	# read configure file
	cp = ConfigParser()
	cp.read("vehicle_data.cfg")
	section = cp.sections()[0]
	wheel_base = float(cp.get(section, "wheel_base"))
	width = float(cp.get(section, "width"))
	dist_from_front_wheel_to_head = float(cp.get(section, "front_wheel_to_head"))
	dist_from_rear_wheel_to_tail = float(cp.get(section, "rear_wheel_to_tail"))
	## pose
	dist_to_left_front_bounds = 0.0
	dist_to_right_front_bounds = 0.0
	dist_to_left_rear_bounds = 0.0
	dist_to_right_rear_bounds = 0.0
	# pose
	pe = PoseEstimator(dist_to_left_front_bounds,
					   dist_to_right_front_bounds,
					   dist_to_left_rear_bounds,
					   dist_to_right_rear_bounds,
					   wheel_base,
					   width,
					   dist_from_front_wheel_to_head,
					   dist_from_rear_wheel_to_tail)
	
	# initialize
	cycles_to_finish_initialization = 5
	initialized = False
	no_cycle_print = False

	while not rospy.is_shutdown():
		#num_l = all_detected_lines_lf.line_number
		#print("*** number of lines: {}".format(num_l))
		#for i in range(num_l):
		#	print(all_detected_lines_lf.all_lines[i].id)

		#if all_detected_lines_lf.line_number > 0:
		#	print(type(all_detected_lines_lf.all_lines[0].positionx))
		
		# update
		ldpp(all_detected_lines_lf,
			 all_detected_lines_rf,
			 all_detected_lines_lr,
			 all_detected_lines_rr)
		
		ldpp.update_ego_lane_lines()

		if not initialized:
			if ldpp.is_left_front_ego_lane_detected and ldpp.is_right_front_ego_lane_detected and ldpp.is_left_rear_ego_lane_detected and ldpp.is_right_rear_ego_lane_detected:
				if cycles_to_finish_initialization == 0:
					initialized = True
					print("Initialized.")
				else:
					cycles_to_finish_initialization -= 1
			else:
				if not no_cycle_print:
					print("Initializing...")
					no_cycle_print = True

		else:
			ldpp.update_intercepts_with_ego_lane_lines()
			ldpp.update_all_distances_to_ego_lane_lines()
			pe(ldpp.distance_from_head_to_left_front_ego_lane_line,
		   	   ldpp.distance_from_head_to_right_front_ego_lane_line,
		   	   ldpp.distance_from_wheel_to_left_rear_ego_lane_line,
		   	   ldpp.distance_from_wheel_to_right_rear_ego_lane_line)
			'''
			dev_direct, dev_angle, dist_on_x_axis_from_vehicle_front_end_center_to_lane_center, dist_on_x_axis_from_vehicle_rear_end_center_to_lane_center, pose_type = pe.estimate_pose()

			print('dev_angle: {}'.format(dev_angle))
			'''
			#print(ldpp.distance_from_head_to_left_front_ego_lane_line, ldpp.distance_from_head_to_right_front_ego_lane_line)
			#print(ldpp.distance_from_wheel_to_left_rear_ego_lane_line, ldpp.distance_from_wheel_to_right_rear_ego_lane_line)
			#print("**************************")
		rate.sleep()

	rospy.spin()	


if __name__ == '__main__':
	listener()
