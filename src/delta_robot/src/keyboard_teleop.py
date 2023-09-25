#!/usr/bin/env python



'''
	Erik Plesko
	16.6.2021

	Help:
		keyboard teleop - uses open cv window and keypresses to control
		broadcasted velocities (array of 5 floats (as for 5 joints), in range -1, 1)

		q 		-> quit
		w / s	-> raise / lower the value by velocityIncrement
		1 - 5 	-> set the active joint (joint of which you set the speed with w/s)
		space	-> stop - set all of the velocities to zero.




'''




import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


import numpy as np
import cv2

width = 512
height = 512
frame = np.zeros((height, width))



# User settings:

nodeName = 'keyboard_teleop'
topicName = '/omegas_control'
nodeRate = 500	# in Hz
velocityIncrement = 1	# increment for which to change the velocity
velocityMax = 100

	
# Initialize:
cv2.imshow('Frame', frame)


def talker():
	pub = rospy.Publisher(topicName, numpy_msg(Floats),queue_size=10)
	rospy.init_node(nodeName)
	r = rospy.Rate(nodeRate) # hz
	
	# Initialization:
	velocity = 0	# 0-1
	activeJoint = 0		# valid joints 0 - 4
	velocities_control = np.zeros((5), dtype=np.float32)
	

	while not rospy.is_shutdown():



		pressedKey = 0xFF & cv2.waitKey(1)

		if pressedKey == ord('q'):				# quit
			break
		elif pressedKey ==  ord('w'):			# increase velocity
			velocity += velocityIncrement
			if velocity > velocityMax:
				velocity = velocityMax
		elif pressedKey ==  ord('s'):			# decrease velocity
			velocity -= velocityIncrement
			if velocity < -velocityMax:
				velocity = -velocityMax
		elif pressedKey ==  32:	# space 		# stop (all velocities to 0)
			velocity = 0
			velocities_control[:] = 0
		elif 49 <= pressedKey < 49 + 5:			# set active joint (keys 1 - 5)
			activeJoint = pressedKey - 49
			velocity = velocities_control[activeJoint]
			# print('Active joint set to:', activeJoint)
		elif pressedKey == 255:
			# no key pressed
			pass
		else:
			print('Unassigned key pressed:')
			print('Num:', pressedKey, 'char:', chr(pressedKey))


		velocities_control[activeJoint] = velocity
		print('Omegas_control in range [-1, 1]:			Active joint: {}'.format(activeJoint + 1))
		print(velocities_control)
		pub.publish(velocities_control)
		r.sleep()

if __name__ == '__main__':
	talker()




























print('Done')