#!/usr/bin/env python3


'''
	Matic Vodopivec
	29.6.2022

	Help:
	Drawing position of the track robot.

'''


import rospy
from geometry_msgs.msg import Pose2D

import numpy as np
import cv2

class tracks_pose:
	def __init__(self, displayResult=False):
		# Initialization:
		self.height = 800
		self.width = 800
		self.frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
		self.x, self.y = 0, 0
		self.z = 5

		#self.pose = Pose2D()

		# Initialize frame:
		#cv2.imshow('Frame', self.frame)

		rospy.init_node('tracks_pose', anonymous=False)
		rospy.Subscriber("/tracks/pose", Pose2D, self.callback_pose)

	def callback_pose(self, data):
		
		self.pose = data
		print(self.pose)
		self.x = self.pose.x * 100
		self.y = self.pose.y * 100

		x_frame = int(self.y + self.width/2)
		y_frame = int(self.x + self.height/2)

		#x_frame = 100
		#y_frame = 100
		self.frame = cv2.circle(self.frame,(x_frame,y_frame), int(self.z), (0,255,0), -1)


		print(x_frame)
		print(y_frame)

		cv2.startWindowThread()
		cv2.imshow('Frame', self.frame)
		cv2.waitKey(0)

		#pressedKey = 0xFF & cv2.waitKey(1)

		
		#cv2.destroyAllwindows() 

		#cv2.waitKey(0) 

		

	def run(self):
		rospy.Subscriber("/tracks/pose", Pose2D, self.callback_pose)

		rospy.spin()



if __name__ == "__main__":

	tracks_pose_var = tracks_pose()
	tracks_pose_var.run()