#!/usr/bin/python
# -*- coding: utf-8 -*-

# standard libraries
import numpy as np
import rospy
import time
import copy
import os


def callback_steering_angle(data_steering_angle):
	global steering_angle
	steering_angle = data_steering_angle

def callback_steering_speed(data_steering_speed):
	global steering_speed
	steering_speed = data_steering_speed

def callback_torque(data_torque):
	global torque
	torque = data_torque

def callback_brake(data_brake):
	global brake
	brake = data_brake

def listener():
	rospy.init_node("vehicle_controller", anonymous=True)

	# subscribers
	# steering related
	rospy.Subscriber("/deployed_steering_angle", Float32, callback_steering_angle, queue_size=1)
	rospy.Subscriber("/deployed_steering_speed", Float32, callback_steering_speed, queue_size=1)
	# torque & brake
	rospy.Subscriber("/deployed_torque", Float32, callback_torque, queue_size=1)
	rospy.Subscriber("/deployed_brake", Float32, callback_brake, queue_size=1)

	# publishers
	pub_steering = rospy.Publisher("/control/steering_angle", Float32, queue_size=1)
    pub_steering_speed_limit = rospy.Publisher("/control/steer_speed_limit", Float32, queue_size=1)
	pub_torque = rospy.Publisher("/torque", Float32, queue_size=1)
	pub_brake = rospy.Publisher("/brake", Float32, queue_size=1)

	rate = rospy.Rate(10)

	#Float32MultiArray().data

	# steering angle
	global steering_angle
	steering_angle = Float32()
	# steering angle speed
	global steering_speed
	steering_speed = Float32()

	# torque
	global torque
	torque = Float32()
	# brake
	brake = Float32()

	while not rospy.is_shutdown():	
		rate.sleep()

	rospy.spin()


if __name__ == '__main__':
	listener()
	



