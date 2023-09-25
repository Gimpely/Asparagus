#!/usr/bin/env python3


'''
	Matic Vodopivec
	13.6.2022

	Help:
	Moving delta robot with Logitech F710 gamepad.

'''


from re import I, S
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Pose2D
import time
from std_msgs.msg import Bool

import numpy as np
import matplotlib.pyplot as plt 
import cv2

from supportingFunctions.minJerkInterpolator import minJerkInterpolator
from supportingFunctions.hold_robot_pose import hold_robot_poz

class joystick_control:
	def __init__(self, boundries, displayResult=False):
		self.boundries = boundries

		# Read max velocity
		self.maxVel = rospy.get_param('/robot_max_vel')
		self.rotVel = rospy.get_param('/robot_rot_vel')
		
		self.start_time = time.time()
		self.time_flag = False
		self.dt = 0.001
		self.gripper_btn_old = 0
		self.close_gripper = False

		self.gripperClosedPosition = 0
		self.gripperOpenedPosition = -120

		# Set start position of the robot
		self.points_X = 0
		self.points_Y = 0
		self.points_Z = -700
		self.point_rot_z = 0
		self.gripper_poz = 0
		self.total_time = 0

		self.joy = Joy()
		self.joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

		self.tracks_twist = TwistStamped()

		self.setVel = self.maxVel / 2

		self.tracks_pose = Pose2D()
		self.save_pose_flag = False

		self.move_gripper = False

		# User settings:
		nodeName = 'delta_joy_control'
		self.topicName = '/r_control'
		nodeRate = 1000	# in Hz
		dT_ms = int(1000/nodeRate)

		rospy.init_node(nodeName)
		#rate = rospy.Rate(5)

		# initialize Interpolator:
		self.maxSpeed = np.array([self.maxVel*10, self.maxVel*10, self.maxVel*10, self.rotVel*10, 10000])
		self.interpolator = minJerkInterpolator(maxSpeed=self.maxSpeed, dT=dT_ms*0.001, printAll=False)

		rospy.Subscriber("joy", Joy, self.callback_joy)

		# Get twist of the tracks
		rospy.Subscriber("/tracks/twist_state", TwistStamped, self.callback_tracks_twist)
		# Get pose 2D of the tracks
		rospy.Subscriber("/tracks/pose", Pose2D, self.callback_tracks_pose)

		#self.pub          = rospy.Publisher(self.topicName, numpy_msg(Floats),queue_size=1)
		self.pub          = rospy.Publisher(self.topicName, Floats,queue_size=1)
		self.pub_zero     = rospy.Publisher("/robot/zero", Bool,queue_size=1)
		
		
		self.home_gripper = False

	def callback_joy(self, data):
		self.joy = data
		#print(self.joy)
		#self.generate_points()

	def callback_tracks_twist(self, data):
		self.tracks_twist = data

	def callback_tracks_pose(self, data):
		self.tracks_pose = data

	def generate_points(self):
		# Determine velocities in all directions
		# Decrease and increase velocity
		if self.joy.buttons[6]:
			self.setVel = self.setVel - 50 * self.dt
			if self.setVel < 1:
				self.setVel = 1
		elif self.joy.buttons[7]:
			self.setVel = self.setVel + 50 * self.dt
			if self.setVel > self.maxVel:
				self.setVel = self.maxVel
		
		#print(self.setVel)
		

		# Check if deadman switch is pressed on gamepad
		deadman_switch = self.joy.buttons[5]
		#print(deadman_switch)
		if deadman_switch:
			# X axis 
			vel_X = self.joy.axes[1] * self.setVel
			#print(self.joy.axes[0])
			# Y axis
			vel_Y = self.joy.axes[0] * self.setVel
			# Z axis
			vel_Z = self.joy.axes[7] * self.setVel
			# Rotation around Z axis
			rot_Z = self.joy.axes[6] * self.rotVel
			# Opening gripper
			rot_gripper = 0
			if self.joy.axes[2] == -1 and self.joy.axes[5] == 1:
				rot_gripper = self.joy.axes[2] * self.rotVel
			elif self.joy.axes[2] == 1 and self.joy.axes[5] == -1:
				rot_gripper = self.rotVel
			#print(rot_gripper)
			

			# Open/close gripper
			if self.joy.axes[2] == -1.0 or self.joy.axes[5] == -1.0:
				gripper_btn = 1
				
				
			else:
				gripper_btn = 0
				

			# opening and closing gripper
			if gripper_btn==1 and self.gripper_btn_old==0:
				self.move_gripper = True
				
				self.close_gripper = not(self.close_gripper)
			self.gripper_btn_old = gripper_btn
		else:
			vel_X = 0
			vel_Y = 0
			vel_Z = 0
			rot_Z = 0
			rot_gripper = 0
			gripper_btn = 0
			self.gripper_btn_old = 0
			

		
		if self.move_gripper:
			self.move_gripper = False
			if self.close_gripper:
				self.gripper_poz = self.gripperClosedPosition
			else:
				self.gripper_poz = self.gripperOpenedPosition

		# Generate points
		self.points_X = self.points_X + vel_X * self.dt
		self.points_Y = self.points_Y + vel_Y * self.dt
		self.points_Z = self.points_Z + vel_Z * self.dt
		self.point_rot_z = self.point_rot_z + rot_Z * self.dt
		self.gripper_poz = self.gripper_poz + rot_gripper * self.dt
		
		# Check if robot holding function is active (if left botom button is pressed)
		current_poz = np.array([self.points_X, self.points_Y, self.points_Z, self.point_rot_z, self.gripper_poz])
		print("current_poz: ", current_poz)
		if self.joy.axes[2] == -1 and deadman_switch:
			# Calculate relative velocity of the delta robot 

			# Transform velocity from m/s to mm/s
			linear_velocity = self.tracks_twist.twist.linear.x * 1000
			angular_velocity = -self.tracks_twist.twist.angular.z
			#print(angular_velocity)
			new_poz = hold_robot_poz(current_poz, linear_velocity, angular_velocity, self.dt)

			#self.points_Y = new_poz[1]
			#self.points_X = new_poz[0]
			# print(new_poz)

		# Set limits in x direction
		if self.points_X < self.boundries[0][0]:
			# Negative limit
			print("yes")
			self.points_X = self.boundries[0][0]
		elif self.points_X > self.boundries[0][1]:
			# Positive limit
			self.points_X = self.boundries[0][1]

		# Set limits in y direction
		if self.points_Y < self.boundries[1][0]:
			# Negative limit
			self.points_Y = self.boundries[1][0]
		elif self.points_Y > self.boundries[1][1]:
			# Positive limit
			self.points_Y = self.boundries[1][1]

		# Set limits in z direction
		if self.points_Z < self.boundries[2][0]:
			# Negative limit
			self.points_Z = self.boundries[2][0]
		elif self.points_Z > self.boundries[2][1]:
			# Positive limit
			self.points_Z = self.boundries[2][1]

		# Set limits in rotation around z axis
		if self.point_rot_z < self.boundries[3][0]:
			# Negative limit
			self.point_rot_z = self.boundries[3][0]
		elif self.point_rot_z > self.boundries[3][1]:
			# Positive limit
			self.point_rot_z = self.boundries[3][1]

		# Set limits for gripper
		if self.gripper_poz < self.boundries[4][0]:
			# Negative limit
			self.gripper_poz = self.boundries[4][0]
		elif self.gripper_poz > self.boundries[4][1]:
			# Positive limit
			self.gripper_poz = self.boundries[4][1]

		
		
		# Generate next point in trajectory
		next_point = np.array([self.points_X, self.points_Y, self.points_Z, self.point_rot_z, self.gripper_poz])
		#print(self.points_Z)
		#print(self.points_Y)

		self.interpolator.addPoint(next_point)     

		status, r_control = self.interpolator.run()
		

		# Check if homing button was pressed -> left push button
		if self.joy.buttons[9]:
			# Send homing command to PLC for gripper homing
			self.home_gripper = True
			self.point_rot_z = 0
			self.gripper_poz = 0
		else:
			self.home_gripper = False
		
		self.pub_zero.publish(self.home_gripper)

		if status > 0:

			r_control = r_control.astype(dtype=np.float32)

			####### added by sslajpah #######
			r_control_msg = Floats()
			r_control_msg.data = [r_control[0],r_control[1],r_control[2],r_control[3], r_control[4]]
			self.pub.publish(r_control_msg)
			###################################

			#self.pub.publish(r_control)

			x = r_control[0]
			y = r_control[1]   

			self.total_time = self.total_time + self.dt
			if self.total_time > 1:
				self.total_time = 0
				#print(r_control)

			#print(r_control)

		# Measuring cycle time
		self.dt = time.time() - self.start_time
		# Save current time
		self.start_time = time.time()
		
		#self.pub = rospy.Publisher(self.topicName, numpy_msg(Floats),queue_size=1)
		
		#rate = rospy.Rate(5)
		#rospy.spin()


if __name__ == '__main__':
	rospy.set_param('/robot_max_vel', 300)
	rospy.set_param('/robot_rot_vel', 200)

	# Call class
	# Set limits in x, y, z direction and also rotation around z axis
	boundries = [[-210, 170], [-190, 170], [-800, -667], [-180, 180], [-120, 0] ] 
	joy_control = joystick_control(boundries=boundries)
	rate = rospy.Rate(500)

	while not rospy.is_shutdown():
		joy_control.generate_points()
		rate.sleep()


	
# -276 170 LEVO
# -75 -190 DESNO
#  198 30 Naprej
#  - 336 25 Nazaj
