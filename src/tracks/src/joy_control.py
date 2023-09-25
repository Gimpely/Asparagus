#!/usr/bin/env python3


'''
	Matic Vodopivec
	13.6.2022

	Help:
	Moving tracks with Logitech F710 gamepad.

'''


from re import I, S
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import time
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from beckhoff_msgs.msg import CmdTracks

import numpy as np
import matplotlib.pyplot as plt 
import cv2


class joystick_control:
	def __init__(self, displayResult=False):

		# Read max velocity
		self.maxVel    = rospy.get_param('/tracks_max_vel')
		self.maxRotVel = rospy.get_param('/tracks_max_rot_vel')
		
		self.start_time = time.time()
		self.dt = 0.001
		
		self.total_time = 0

		self.joy = Joy()
		self.joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

		self.setVel = self.maxVel / 2
		self.rotVel = self.maxRotVel / 3
		self.slowdown = 0
		self.start_movement_btn_old = 0
		self.linear_movement = False

		self.reset_odom = False

		# Define twist
		self.twist = TwistStamped()

		# User settings:
		nodeName = 'track_joy_control'
		self.topicName = '/tracks/twist_cmd'

		rospy.init_node(nodeName)

		rospy.Subscriber("joy", Joy, self.callback_joy)
		# Read commands from motion planning program
		self.cmdTracks = CmdTracks()
		rospy.Subscriber('/tracks/cmd', CmdTracks, self.callback_cmd_tracks)
		rospy.Subscriber('/asparagus_cluster', Int32, self.adjust_speed)
		self.pub = rospy.Publisher(self.topicName, TwistStamped,queue_size=1)
		self.pub_reset_odom = rospy.Publisher("/tracks/reset_odom", Bool,queue_size=1)

	def callback_joy(self, data):
		self.joy = data

	def callback_cmd_tracks(self, data):
		self.cmdTracks = data
		#print(self.cmdTracks)

	def adjust_speed(self, data):
		num_of_asparagus_in_a_cluster = data.data
		if num_of_asparagus_in_a_cluster == 0:
			self.slowdown = 0
		elif num_of_asparagus_in_a_cluster == 1:
			self.slowdown = 1
		elif num_of_asparagus_in_a_cluster == 2:
			self.slowdown = 2

		
		# print("::::::::::::::::",self.num_of_asparagus )

		

	def generate_points(self):
		# Determine velocities in all directions
		# Decrease and increase linear velocity (Y and A button on joystick)
		if self.joy.buttons[0]:
			self.setVel = self.setVel -  0.05* self.dt
			if self.setVel < 0:
				self.setVel = 0
		elif self.joy.buttons[3]:
			self.setVel = self.setVel + 0.05* self.dt
			if self.setVel > self.maxVel:
				self.setVel = self.maxVel

		# Decrease and increase angular velocity (X and B button on joystick)
		if self.joy.buttons[2]:
			self.rotVel = self.rotVel -  0.05* self.dt
			if self.rotVel < 0:
				self.rotVel = 0
		elif self.joy.buttons[1]:
			self.rotVel = self.rotVel + 0.05* self.dt
			if self.rotVel > self.maxRotVel:
				self.rotVel = self.maxRotVel
		
		#print(self.rotVel)
		

		# Check if deadman switch is pressed on gamepad
		deadman_switch = self.joy.buttons[5]
		#print(deadman_switch)
		if deadman_switch:
			# Linear velocity
			linear_velocity = self.joy.axes[4] * self.setVel
			#print(linear_velocity)
			# Angular velocity
			angular_velocity = self.joy.axes[3] * self.rotVel
			# Move track with constant linear velocity
			start_movement_btn = self.joy.buttons[4]
			if start_movement_btn==1 and self.start_movement_btn_old==0:
				self.linear_movement = not(self.linear_movement)
			self.start_movement_btn_old = start_movement_btn
		else:
			linear_velocity = 0
			angular_velocity = 0
			
			start_movement_btn = self.joy.buttons[4]
			if start_movement_btn==1 and self.start_movement_btn_old==0:
				self.linear_movement = False
			self.start_movement_btn_old = start_movement_btn
			
		# Check if we received movement cmd from motion control program
		if self.cmdTracks.stop_tracks:
			linear_velocity = 0
			self.linear_movement = False

		# Check if constant movement is active
		if self.linear_movement:
			# linear_velocity = self.setVel
			if self.slowdown == 0:
				linear_velocity = self.setVel * 0.6
			if self.slowdown == 1:
				linear_velocity = self.setVel * 0.3
			if self.slowdown == 2:
				linear_velocity = self.setVel * 0
			# print(linear_velocity)

		# Measuring cycle time
		self.dt = time.time() - self.start_time
		# Save current time
		self.start_time = time.time()

		# Send timestamp
		self.twist.header.stamp = rospy.get_rostime()
		# Send velocity to PLC
		self.twist.twist.linear.x = linear_velocity
		self.twist.twist.angular.z = angular_velocity

		#print(self.twist.twist)

		self.pub.publish(self.twist)
		
		self.pub = rospy.Publisher(self.topicName, TwistStamped,queue_size=1)

		# Reset odometry -> right push button
		reset_odom_old = self.reset_odom
		if self.joy.buttons[10]:
			# Send homing command to PLC for gripper homing
			self.reset_odom = True
		else:
			self.reset_odom = False

		if	~reset_odom_old & self.reset_odom:
			print('Odometry reset.')
		
		self.pub_reset_odom.publish(self.reset_odom)



if __name__ == '__main__':
	rospy.set_param('/tracks_max_vel', 0.2)
	rospy.set_param('/tracks_max_rot_vel', 0.1)

	# Call class
	# Set limits in x, y, z direction and also rotation around z axis
	joy_control = joystick_control()
	rate = rospy.Rate(500)

	while not rospy.is_shutdown():
		joy_control.generate_points()
		rate.sleep()
