#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy

import numpy as np
import cv2

width = 512
height = 512
frame = np.zeros((height, width))



# User settings:
refreshTime = 100	# time in ms














topicName = 'floats'
nodeName = 'talker'
nodeRate = 10	# in Hz



def talker():
	pub = rospy.Publisher(topicName, numpy_msg(Floats),queue_size=10)
	rospy.init_node(nodeName, anonymous=True)
	r = rospy.Rate(nodeRate) # hz
	
	
	
	# Initializing:
	velocity = 0	# 0-1
	cv2.imshow('Frame', frame)

	while not rospy.is_shutdown():
		a = numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32)



		pressedKey = 0xFF & cv2.waitKey(refreshTime)
		print(pressedKey)

		if pressedKey == ord('q'):
			break
		elif pressedKey ==  ord('w'):
			velocity += 0.1
			if velocity > 1:
				velocity = 1
		elif pressedKey ==  ord('s'):
			velocity -= 0.1
			if velocity < -1:
				velocity = -1
		elif pressedKey ==  32:		# space pressed
			velocity = 0
		else:
			print('Unassigned key pressed:')
			print('Num:', pressedKey, 'char:', chr(pressedKey))



		print('Velocity =', velocity)




		
		pub.publish(a)
		r.sleep()

if __name__ == '__main__':
	talker()




























print('Done')