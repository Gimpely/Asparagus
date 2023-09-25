#!/usr/bin/env python



'''
	Erik Plesko
	21.6.2021

	Help:
	


'''




import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from trajectory_msgs.msg import JointTrajectory


import numpy as np
import cv2

width = 512
height = 512
frame = np.zeros((height, width))



# User settings:

nodeName = 'testPoints'
topicName = 'points'
nodeRate = 0.2	# in Hz
velocityIncrement = 1	# increment for which to change the velocity
velocityMax = 100

	
# Initialize:
cv2.imshow('Frame', frame)


def talker():
	pub = rospy.Publisher(topicName, JointTrajectory, queue_size=10)
	rospy.init_node(nodeName)
	r = rospy.Rate(nodeRate) # hz
	
	# Initialization:
	velocity = 0	# 0-1
	activeJoint = 0		# valid joints 0 - 4
	velocities_control = np.zeros((5), dtype=np.float32)


	testMessage = JointTrajectory()

	print(testMessage)
	print(testMessage.header)
	print('-----------------')
	

	while not rospy.is_shutdown():



		pressedKey = 0xFF & cv2.waitKey(0)

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
		pub.publish(testMessage)
		r.sleep()

if __name__ == '__main__':
	talker()

