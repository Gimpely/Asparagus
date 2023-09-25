#!/usr/bin/env python3



'''
	Erik Plesko
	22.6.2021

	Help:

	main part: 
	
	r - run (moves from point to point - points from preSavedPoints are taken)
	numbers 0-9 - move to point preSavedPoints[num]


	TODO:
	beside other keys:

	Keys to move in cartesian:
	awsd - in the x-y plane
	e,x - up, down
	g, t - grip, open gripper
	y,h - 4th joint (phi) y-> positive phi
	

'''




import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time

import numpy as np
import matplotlib.pyplot as plt 
import cv2


from supportingFunctions.minJerkInterpolator import minJerkInterpolator

# Frame settings:
width = 512
height = 512

# User settings:

nodeName = 'moveL_control'
topicName = '/r_control'
nodeRate = 500	# in Hz
dT_ms = int(1000/nodeRate)

print(dT_ms)



#_____________________________________________________________________
# ### Initialization ###

# initialize Interpolator:
maxSpeed = np.array([200, 200, 200, 400, 400])
#dT_0 = 0.002
interpolator = minJerkInterpolator(maxSpeed=maxSpeed, dT=0.002, printAll=False)


def talker():
	# Initialization:
	frame = np.zeros((height, width, 3), dtype=np.uint8)
	x, y = 0, 0
	z = 5
	# Initialize frame:
	cv2.imshow('Frame', frame)

	gripperClosedPosition = 0
	gripperOpenedPosition = -120
	phi = 0




	preSavedPoints_calib = np.array([
		[0,		0, 		-700, 	phi, 	gripperOpenedPosition],
		[0,		0, 		-750, 	phi, 	gripperOpenedPosition],
		[200,	0,	 	-750, 	phi, 	gripperOpenedPosition],
		[0,	200,	 	-750, 	phi, 	gripperOpenedPosition],
		[-0,	0,	-700, 	phi, 	gripperOpenedPosition],
		[-0,	0,	-800, 	phi, 	gripperOpenedPosition],
		[-0,	0,	-810, 	phi, 	gripperOpenedPosition],
		[-0,	0,	-820, 	phi, 	gripperOpenedPosition],
	])


	preSavedPoints_tuning = np.array([
		[0,		0, 		-700, 	phi, 	gripperOpenedPosition],
		[-100,	0, 		-700, 	phi, 	gripperOpenedPosition],
		[100,	0,	 	-700, 	phi, 	gripperOpenedPosition],
		[0,	-100,		-700, 	phi,	gripperOpenedPosition],
		[0,	100, 		-700, 	120, 	gripperOpenedPosition],
		[-0,	0,		-700, 	phi, 	gripperOpenedPosition],
		[-0,	0,		-700, 	phi, 	gripperClosedPosition],
	])


	preSavedPoints_Old = np.array([
		[0,		100, 		-650, 	phi, 	gripperOpenedPosition],
		[-50,	100,	 	-650, 	phi, 	gripperOpenedPosition],
		[-50,	0,	 		-750, 	phi, 	gripperOpenedPosition],
		[-50,	0,	 		-750, 	phi, 	gripperClosedPosition],
		[-50,	100,	 	-650, 	phi, 	gripperClosedPosition],

		# Spusti v kosaro:
		[200,	50,	 	-650, 	phi, 	gripperClosedPosition],
		[200,	50,	 		-650, 	phi, 	gripperOpenedPosition],
		[200,	0,	 		-650, 	phi, 	gripperOpenedPosition],
		[200,	50,	 	-650, 	phi, 	gripperOpenedPosition],

		[100,	100,	 	-650, 	phi, 	gripperOpenedPosition],
		[0,		100, 		-650, 	phi, 	gripperOpenedPosition],

		# Primi na tocki 2
		[100,	100,	 	-650, 	phi, 	gripperOpenedPosition],
		[100,	0,	 		-750, 	phi, 	gripperOpenedPosition],
		[100,	0,	 		-750, 	phi, 	gripperClosedPosition],
		[100,	100,	 	-650, 	phi, 	gripperClosedPosition],


		# Spusti v kosaro:
		[200,	50,	 	-650, 	phi, 	gripperClosedPosition],
		[200,	50,	 		-650, 	phi, 	gripperOpenedPosition],
		[200,	0,	 		-650, 	phi, 	gripperOpenedPosition],
		[200,	50,	 	-650, 	phi, 	gripperOpenedPosition],

		# Odlozi na tocki 1
		# [-50,	100,	 	-650, 	phi, 	gripperClosedPosition],
		# [-50,	0,	 		-750, 	phi, 	gripperClosedPosition],
		# [-50,	0,	 		-750, 	phi, 	gripperOpenedPosition],
		# [-50,	100,	 	-650, 	phi, 	gripperOpenedPosition],

		[0,		100, 		-650, 	phi, 	gripperOpenedPosition],



		# [ 0,	0,	 	-700, 	0, 	0],
		# [ 50,	0, 		-700, 	0, 	0],
		# [-0,	-100, 	-700, 	0, 	0],
		# [-0,	100, 	-700, 	0, 	0],
	])

	preSavedPoints = preSavedPoints_tuning


	lastPoint = preSavedPoints[0]
	movingStopped = False
	gripperClosed = True
	pressedNum = 0
	moveByAxis = True


	pub = rospy.Publisher(topicName, numpy_msg(Floats),queue_size=10)
	rospy.init_node(nodeName)
	

	while not rospy.is_shutdown():


		pressedKey = 0xFF & cv2.waitKey(dT_ms)

		if pressedKey == ord('q'):				# quit
			break
		elif 48 <= pressedKey < 48 + 9:               # key 
			pressedNum = pressedKey - 48
			print('number:', pressedNum)
			if pressedNum <= preSavedPoints.shape[0]:
				lastPoint = np.copy(preSavedPoints[pressedNum])
				# if gripperClosed:
				# 	lastPoint[4] = gripperClosedPosition
				# else:
				# 	lastPoint[4] = gripperOpenedPosition

				interpolator.addPoint(lastPoint)
				print('point {} added to the fifo'.format(lastPoint))
			else:
				print('Point {} doesnt exist.'.format(pressedNum))
		
		elif pressedKey ==  ord('t'):			# open Gripper
			gripperClosed = False
			lastPoint[4] = gripperOpenedPosition
			interpolator.addPoint(lastPoint)
			print('point {} added to the fifo'.format(lastPoint))
		elif pressedKey ==  ord('g'):			# grip (close gripper)
			gripperClosed = True
			lastPoint[4] = gripperClosedPosition
			interpolator.addPoint(lastPoint)
			print('point {} added to the fifo'.format(lastPoint))
		elif pressedKey ==  32:	# space 		# stop 
			movingStopped = True
			interpolator.printPoints()
			print('Moving stopped! (any key to continue)')


			cv2.waitKey(0)
		elif pressedKey ==  ord('r'):			# resume (run)
			movingStopped = False


			for point in preSavedPoints:
				interpolator.addPoint(point)

		elif pressedKey ==  ord('c'):			# clear points
			interpolator.clearPoints()
			print('Points cleared')	
			
						
		elif pressedKey ==  ord('m'):			# clear points
			moveByAxis = not moveByAxis
			if moveByAxis:
				print('Move by axis selected')
			else:
				print('Move by points selected')
		elif pressedKey == 255:
			# no key pressed
			pass
		else:
			print('Unassigned key pressed:')
			print('Num:', pressedKey, 'char:', chr(pressedKey))



		status, r_control = interpolator.run()

		if status > 0:

			r_control = r_control.astype(dtype=np.float32)
			pub.publish(r_control)

			x = r_control[0]
			y = r_control[1]


		frame[:] = 0

		x_frame = int(x + width/2)
		y_frame = int(-y + height/2)
		frame = cv2.circle(frame,(x_frame,y_frame), int(z), (0,255,0), -1)

		cv2.imshow('Frame', frame)




if __name__ == '__main__':
	talker()


