import os, sys
import numpy as np
import cv2
import time
import copy
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int8
from cqpilot_msg.msg import mmt_line
from cqpilot_msg.msg import lines
from pose_estimator import PoseEstimator
from configparser import ConfigParser
import json

from lane_detection_process_pipeline import LaneDetectionProcessPipeline
from vehicle_lane_position_secondary_process import check_validity_of_all_distances_to_ego_lane_lines

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

def callback_vehicle_speed(speed):
	global c_speed
	c_speed = speed


def listener():
	rospy.init_node("show_vehicle_pose_in_lane", anonymous=True)
	
	rospy.Subscriber("/detection/lines/deadzone_lf", lines, callback_detected_left_front_lane_lines, queue_size=1)
	rospy.Subscriber("/detection/lines/deadzone_rf", lines, callback_detected_right_front_lane_lines, queue_size=1)
	rospy.Subscriber("/detection/lines/deadzone_lr", lines, callback_detected_left_rear_lane_lines, queue_size=1)
	rospy.Subscriber("/detection/lines/deadzone_rr", lines, callback_detected_right_rear_lane_lines, queue_size=1)
	rospy.Subscriber("/can5/speed", Float32, callback_vehicle_speed, queue_size=1)

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
	# vehicle speed
	global c_speed
	c_speed = Float32(0.0)

	# left front ref 
	with open('./dat/lf_ref.json', 'r') as f:
		left_front_ref = json.load(f)
	# right front ref
	with open('./dat/rf_ref.json', 'r') as f:
		right_front_ref = json.load(f)
	# left rear ref
	with open('./dat/lr_ref.json', 'r') as f:
		left_rear_ref = json.load(f)
	# right rear ref
	with open('./dat/rr_ref.json', 'r') as f:
		right_rear_ref = json.load(f)

	# left front pix2real map
	with open('./dat/pix2real_lf.json', 'r') as f:
		left_front_pix_real_pos_map = json.load(f)
	# right front pix2real map
	with open('./dat/pix2real_rf.json', 'r') as f:
		right_front_pix_real_pos_map = json.load(f)
	# left rear pix2real map
	with open('./dat/pix2real_lr.json', 'r') as f:
		left_rear_pix_real_pos_map = json.load(f)
	# right rear pix2real map
	with open('./dat/pix2real_rr.json', 'r') as f:
		right_rear_pix_real_pos_map = json.load(f)
	
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

	# size of window of historical data
	window_size = 10 # FIXME
	historical_distances_from_head_to_left_front_ego_lane_line = []
	historical_distances_from_head_to_right_front_ego_lane_line = []
	historical_distances_from_wheel_to_left_rear_ego_lane_line = []
	historical_distances_from_wheel_to_right_rear_ego_lane_line = []

	# last moment
	last_moment = time.time()

	while not rospy.is_shutdown():
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

			if ldpp.distance_from_head_to_left_front_ego_lane_line==None:
				historical_distances_from_head_to_left_front_ego_lane_line = []
			if ldpp.distance_from_head_to_right_front_ego_lane_line==None:
				historical_distances_from_head_to_right_front_ego_lane_line = []
			if ldpp.distance_from_head_to_left_rear_ego_lane_line==None:
				historical_distances_from_head_to_left_rear_ego_lane_line = []
			if ldpp.distance_from_head_to_right_rear_ego_lane_line==None:
				historical_distances_from_head_to_right_rear_ego_lane_line = []

			# check the validity of updated distances
			lf_maybe_outlier, rf_maybe_outlier, lr_maybe_outlier, rr_maybe_outlier, assessed_lf, assessed_rf, assessed_lr, assessed_rr = vehicle_lane_position_secondary_process.check_validity_of_all_distances_to_ego_lane_lines(c_speed,
												 pe.dev_angle,
												 ldpp.distance_from_head_to_left_front_ego_lane_line,
												 ldpp.distance_from_head_to_right_front_ego_lane_line,
												 ldpp.distance_from_wheel_to_left_rear_ego_lane_line,
												 ldpp.distance_from_wheel_to_right_rear_ego_lane_line,
												 historical_distances_from_head_to_left_front_ego_lane_line,
												 historical_distances_from_head_to_right_front_ego_lane_line,
												 historical_distances_from_wheel_to_left_rear_ego_lane_line,
												 historical_distances_from_wheel_to_right_rear_ego_lane_line)

			current_moment = time.time()
			elapsed_time = current_moment - last_moment

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
		# update last moment
		last_moment = current_moment

	rospy.spin()	


if __name__ == '__main__':
	listener()
