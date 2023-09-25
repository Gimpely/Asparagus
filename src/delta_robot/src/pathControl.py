#!/usr/bin/env python



'''
	Erik Plesko
	18.6.2021

	Help:
	




'''




import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time

import numpy as np
import matplotlib.pyplot as plt 
import cv2

width = 512
height = 512
frame = np.zeros((height, width), dtype=np.uint8)



# User settings:

nodeName = 'path_control'
topicName = 'r_control'
nodeRate = 100	# in Hz

velocityIncrement = 1	# increment for which to change the velocity
velocityMax = 100
travelVelocity = 300		# mm/s

moveCenterX = 50
moveCenterY = 50

	
# Initialize:
cv2.imshow('Frame', frame)


def talker():
	global frame

	pub = rospy.Publisher(topicName, numpy_msg(Floats),queue_size=10)
	rospy.init_node(nodeName)
	r = rospy.Rate(nodeRate) # hz
	
	# Initialization:
	velocity = 0	# 0-1
	activeJoint = 0		# valid joints 0 - 4


	while not rospy.is_shutdown():



		pressedKey = 0xFF & cv2.waitKey(1)

		if pressedKey == ord('q'):				# quit
			break
		elif pressedKey == 49:                  # key 1
			# Move of type 1:
			print('Move type 1 - circle')


			N = 600
			N_circles = 10
			radius = 120
			travelVelocity = 100		# mm/s

			dT = 2*np.pi*radius/travelVelocity/N
			print(dT)
			print('path time: {:.02f} s'.format(N*dT))

			# Frequency:
			timer1 = time.time()


			for i in range(N_circles*N):
				x = 1*radius*np.cos(2*np.pi*i/N)
				y = 1*radius*np.sin(2*np.pi*i/N)
				# y = 0.5*radius*np.sin(2*2*np.pi*i/N)
				z = -700 #+ 0.5*radius*np.sin(2*np.pi*i/N)

				x += moveCenterX
				y += moveCenterY


				r_control = np.array([x, y, z, 0, 0], dtype=np.float32)

				pub.publish(r_control)


				print('Frequency: {:.02f}'.format(1/(time.time() - timer1)   ))

				timer1 = time.time()
				

				print(x, y, z)




				x_frame = int(x + width/2)
				y_frame = int(-y + height/2)
				frame[:] = 10
				print(x_frame)
				frame = cv2.circle(frame,(x_frame,y_frame), 3, (255,255,255), -1)
				cv2.imshow('Frame', frame)


				timeToWait = int(dT*1000)
				if timeToWait <= 0:
					timeToWait = 1
				pressedKey = 0xFF & cv2.waitKey(timeToWait)

				if pressedKey == ord('q'):				# stop the path
					print('Path stopped.')
					break
			print('Path ended')
# _____________________________________________________________________________________
		elif pressedKey == 50:                  # key 2
			# Move of type 2:
			print('Move type 2 - square')

			N = 400
			N_rounds = 10
			radius = 100

			travelVelocity = 50		# who knows the units :')


			dT = 0.01
			print(dT)
			print('path time: {:.02f} s'.format(N*dT))

			# Frequency:
			timer1 = time.time()


			for i in range(N_rounds*N):


				if i % N < N/4:
					x = radius*np.cos(2*np.pi*i*2/N)
					y = radius
				elif i % N < N/4*2:
					x = -radius
					y = -radius*np.cos(2*np.pi*i*2/N)
				elif i % N < N/4*3:
					x = -radius*np.cos(2*np.pi*i*2/N)
					y = -radius
				elif i % N < N/4*4:
					x = radius
					y = radius*np.cos(2*np.pi*i*2/N)
				
				x += moveCenterX
				y += moveCenterY

				z = -650 #+ 0.5*radius*np.sin(2*np.pi*i/N)




				r_control = np.array([x, y, z, 0, 0], dtype=np.float32)

				pub.publish(r_control)


				print('Frequency: {:.02f}'.format(1/(time.time() - timer1)   ))

				timer1 = time.time()
				

				print(x, y, z)




				x_frame = int(x + width/2)
				y_frame = int(-y + height/2)
				frame[:] = 10
				print(x_frame)
				frame = cv2.circle(frame,(x_frame,y_frame), 3, (255,255,255), -1)
				cv2.imshow('Frame', frame)


				timeToWait = int(dT*1000)
				if timeToWait <= 0:
					timeToWait = 1
				pressedKey = 0xFF & cv2.waitKey(timeToWait)

				if pressedKey == ord('q'):				# stop the path
					print('Path stopped.')
					break
			print('Path Square ended')
# _____________________________________________________________________________________
		
		elif pressedKey == 51:                  # key 3
			# Move of type 3 ---- short Move:
			print('Move type 3 - short move')

			radius = 80

			dT = 0.004

			timeConst = 1

			N = int(2*timeConst/dT)


			print(dT)

			# Frequency:
			timer1 = time.time()


			for i in range(N):


				if i < N/2:
					x = radius
				else:
					x = -radius
				y = 0
				z = -700 #+ 0.5*radius*np.sin(2*np.pi*i/N)




				r_control = np.array([x, y, z, 0, 0], dtype=np.float32)

				pub.publish(r_control)


				print('Frequency: {:.02f}'.format(1/(time.time() - timer1)   ))

				timer1 = time.time()
				

				print(x, y, z)




				x_frame = int(x + width/2)
				y_frame = int(-y + height/2)
				frame[:] = 10
				print(x_frame)
				frame = cv2.circle(frame,(x_frame,y_frame), 3, (255,255,255), -1)
				cv2.imshow('Frame', frame)


				timeToWait = int(dT*1000)
				if timeToWait <= 0:
					timeToWait = 1
				pressedKey = 0xFF & cv2.waitKey(timeToWait)

				if pressedKey == ord('q'):				# stop the path
					print('Path stopped.')
					break
			print('Path Square ended')
		# --------------------------------------------------------------------

		elif pressedKey == 49+3:                  # key 4
			# Move of type 4:
			print('Move type 4 line')


			N = 600
			N_circles = 10
			radius = 100
			travelVelocity = 800		# mm/s

			dT = 2*np.pi*radius/travelVelocity/N
			print(dT)
			print('path time: {:.02f} s'.format(N*dT))

			# Frequency:
			timer1 = time.time()


			for i in range(N_circles*N):
				x = 0*radius*np.cos(2*np.pi*i/N)
				y = 0*radius*np.sin(2*np.pi*i/N)
				# y = 0.5*radius*np.sin(2*2*np.pi*i/N)
				z = -700 + 0.5*radius*np.sin(2*np.pi*i/N)

				x += moveCenterX
				y += moveCenterY


				r_control = np.array([x, y, z, 0, 0], dtype=np.float32)

				pub.publish(r_control)


				print('Frequency: {:.02f}'.format(1/(time.time() - timer1)   ))

				timer1 = time.time()
				

				print(x, y, z)




				x_frame = int(x + width/2)
				y_frame = int(-y + height/2)
				frame[:] = 10
				print(x_frame)
				frame = cv2.circle(frame,(x_frame,y_frame), 3, (255,255,255), -1)
				cv2.imshow('Frame', frame)


				timeToWait = int(dT*1000)
				if timeToWait <= 0:
					timeToWait = 1
				pressedKey = 0xFF & cv2.waitKey(timeToWait)

				if pressedKey == ord('q'):				# stop the path
					print('Path stopped.')
					break
			print('Path ended')
		# --------------------------------------------------------------------

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
		elif 49 <= pressedKey < 49 + 5:			# set active joint (keys 1 - 5)
			activeJoint = pressedKey - 49
			pass
			# print('Active joint set to:', activeJoint)
		elif pressedKey == 255:
			# no key pressed
			pass
		else:
			print('Unassigned key pressed:')
			print('Num:', pressedKey, 'char:', chr(pressedKey))


		frame[:] = 0
		cv2.imshow('Frame', frame)



		r.sleep()

if __name__ == '__main__':
	talker()


