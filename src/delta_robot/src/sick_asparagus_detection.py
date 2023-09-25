#!/usr/bin/env python



'''
	Erik Plesko
	23.6.2021

	Help:
	
		q - quit
		d - detect
		h - harvest

		c - clear points (from the interpolator)


'''




import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time


from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import LaserScan


import numpy as np
import matplotlib.pyplot as plt 
import cv2


from supportingFunctions.minJerkInterpolator import minJerkInterpolator
import supportingFunctions.asparagusDetection as ad


width = 1000
height = 800



# User settings:

nodeName = 'sick_detection'
# topicName_positions = 'asparagus_positions'
topicName_scan = '/sick_safetyscanners/scan1'
topicName_control = 'r_control'
nodeRate = 100	# in Hz
dT_ms = int(1000/nodeRate)


# TODO: continue this scipt - get laser data, visualize it and try to find asparagus locations.


angleMin = 0
angleMax = 0
ranges = None

def callback_scan(data):
	global angleMin
	global angleMax
	global ranges 

	angleMin = data.angle_min
	angleMax = data.angle_max
	ranges = np.array(data.ranges)*1000


def xyToImage(x, y):
	x_frame = int(x + width/2)
	y_frame = int(-y + height/2)
	return x_frame, y_frame

#_____________________________________________________________________
# ### Initialization ###

# initialize Interpolator:
cSpeed = 200
maxSpeed = np.array([cSpeed, cSpeed, 100, 200, 300])
interpolator = minJerkInterpolator(maxSpeed=maxSpeed, printAll=False)



# Test points:
preSavedPoints_calib = np.array([
		[0,		0, 		100, 	90, 	-110],
		[150,		250, 		70, 	0, 	-110],
		[150,		250, 		60, 	0, 	-110],
		[0,		0, 		100, 	-90, 	-110],
		[200,		0, 		100, 	90, 	-110],
		[200,		0, 		100, 	180, 	-110],
		[200,		0, 		100, 	90, 	-110],
		[200,		0, 		100, 	0, 		-110],
		[200,		0, 		100,		-90, 	-110],
		
	])
preSavedPoints_test = np.array([
		[0, 	100,	200,	90,	-90],
		[0, 	100,	200,	0,	-90],
		[0, 	100,	200,	180,	-90],
		[0, 	100,	200,	90,	-90],
	])

preSavedPoints_via = np.array([
		[-100, 	200,	200,	0,	-110],
		[-100, 	200,	200,	0,	-0],

		[0, 	100,	200,	0,	-90],
		[0, 	100,	200,	180,	-90],

		[200, 	100,	200,	0,	-110],
		[200, 	100,	200,	0,	-0],

		[0, 	100,	200,	90,	-90],
	])

preSavedPoints = preSavedPoints_calib


def talker():
	pub = rospy.Publisher(topicName_control, numpy_msg(Floats), queue_size=10)
	rospy.init_node(nodeName)
	rospy.Subscriber(topicName_scan, LaserScan, callback_scan)


	# Initialization:
	frame = np.zeros((height, width, 3), dtype=np.uint8)
	x, y = 0, 0
	z = 5
	gripper = 0
	asparagusGlobal = []
	# Initialize frame:
	cv2.imshow('Frame', frame)

	homePoint = np.array([0, 	200,	300,	90,	-110])
	viaPoint = [250,	250]
	depositPoint = [150, 250]

	
	interpolator.addPoint(homePoint)


	asparagusPoints = None


	
	while not rospy.is_shutdown():


		
		pressedKey = 0xFF & cv2.waitKey(dT_ms)

		if pressedKey == ord('q'):				# quit
			break
		if pressedKey == ord('t'):				# Test
			for point in preSavedPoints_test:
				interpolator.addPoint(point)
		elif pressedKey == ord('y'):			# Fake asparagus detection (test)
			# Fake asparagus detection:
			asparagusGlobal = [[200,-100], [0,0]]#, [100, 0]]
		elif pressedKey == ord('d'):				# detect asparagus
			# Detect asparagus if data is available:
			print(angleMin, angleMax)
			if ranges is not None:
				asparagusPoints, observedPoints = ad.detectAsparagus(ranges, minAngle=angleMin, maxAngle=angleMax,
							xLim=[200, 700],
							yLim=[0, 600],
							phiOffset=np.pi/2,
							plotResults=True,
							printAll=True
							)

				print('asparagus points')
				print(asparagusPoints)

				# Calibration of CS:
				# T_o - x,y coordinates in laser scan coordinates. Cordinates of the Global CS origin
				# T_x - x,y coordinates in laser scan coordinates. Cordinates of a point on x axis of Global CS (I used x ~ 200mm)
				T_o=[377.36018898,  293.9326781],
				T_x=[376.4866538,   495.11714984],
				asparagusGlobal = ad.transform_ScanToGlobal(asparagusPoints, T_o=T_o, T_x=T_x)
				observedGlobal = ad.transform_ScanToGlobal(observedPoints, T_o=T_o, T_x=T_x)


				if True:
					plt.plot(asparagusGlobal[:,0], asparagusGlobal[:,1], ':gx', markersize=10)

					plt.plot(observedGlobal[:,0], observedGlobal[:,1], '.')

					plt.gca().set_aspect('equal', adjustable='box')
					plt.title('Global cs')
					plt.show()

		elif pressedKey == ord('h'):				# harvest asparagus
			for i, aPoint in enumerate(asparagusGlobal):
				# approach angle:
				if i <= 5:
					approachAngle = np.pi*(1.0/2+0.0) + 0*np.pi/2*(i%2)
					print(approachAngle)
				else: 
					approachAngle = np.pi
				trajPoints, waitTimes = ad.generateHarvestPoints(aPoint, 
									approachAngle=approachAngle
									)

				# if i <= 5:
				# 	interpolator.addPoint(preSavedPoints_via[0])
				
				
				for j, p in enumerate(trajPoints):
					interpolator.addPoint(p, waitAfter=waitTimes[j])
				
				# interpolator.addPoint(viaPoint)
				# not yet implemented


				# if i <= 5:
				# 	interpolator.addPoint(preSavedPoints_via[1])

				trajPoints, waitTimes = ad.generateDepositPoints(depositPoint)
				for j, p in enumerate(trajPoints):
					interpolator.addPoint(p, waitAfter=waitTimes[j])
				
			interpolator.addPoint(homePoint)


			interpolator.printPoints()
		
		elif pressedKey ==  ord('c'):			# clear points
			interpolator.clearPoints()
			print('Points cleared')		
		elif pressedKey ==  32:	# space 		# stop 
			interpolator.printPoints()
			print('Moving stopped! (But after resume the move well be rapid!!!)')
			cv2.waitKey(0)
		elif 48 <= pressedKey < 48 + 9:               # key 
			pressedNum = pressedKey - 48
			print('number:', pressedKey)
			if pressedNum < preSavedPoints.shape[0]:
				lastPoint = np.copy(preSavedPoints[pressedNum])
				# if gripperClosed:
				# 	lastPoint[4] = gripperClosedPosition
				# else:
				# 	lastPoint[4] = gripperOpenedPosition

				interpolator.addPoint(lastPoint)
				print('point {} added to the fifo'.format(lastPoint))
			else:
				print('Point {} doesnt exist.'.format(pressedNum))
		elif pressedKey == 255:
			# no key pressed
			pass
		else:
			print('Unassigned key pressed:')
			print('Num:', pressedKey, 'char:', chr(pressedKey))


		

		status, r_control = interpolator.run()

		if status > 0:

			r_control = r_control.astype(dtype=np.float32)
			# ________________________________________
			# Transformation from global to robot's cs
			zOffset = -775
			r_control[2] += zOffset
			# ----------------------------------------
			pub.publish(r_control)

			x = r_control[0]
			y = r_control[1]
			z = int((r_control[2]-zOffset)/2 + 5)

			phi = np.pi/180*r_control[3]
			phi_G = np.pi - phi
			
			r_tool = 40
			xp = r_tool*np.cos(phi_G) + x
			yp = r_tool*np.sin(phi_G) + y


			gripper = r_control[4]


		# Draw stuff:
		frame[:] = 0
		
		
		for point in asparagusGlobal:
			x_frame = int(point[0] + width/2)
			y_frame = int(-point[1] + height/2)
			
			frame = cv2.circle(frame,(x_frame,y_frame), int(10), (250,100, 250), -1)


		x_frame = int(x + width/2)
		y_frame = int(-y + height/2)
		gripper = int(gripper)


		if status > 0:
			# Draw platform:
			frame = cv2.circle(frame,(xyToImage(xp,yp)), int(z), (0, int(255+2*gripper), int(30-2*gripper)), -1)


		frame = cv2.circle(frame,(x_frame,y_frame), 15, (250,20,20), -1)


		cv2.imshow('Frame', frame)




if __name__ == '__main__':
	talker()

