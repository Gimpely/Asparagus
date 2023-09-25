#!/usr/bin/env  python
import rospy
#from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Vector3, Twist

import socket
import struct
import numpy as np


nodeName = 'UPD_send_omegas'
topicName = 'omegas_control'
# Limit omegas to be sent:
motor1to3Limit = 500	#deg/s
motor4Limit = 1000	#deg/s
motor5Limit = 1000	#deg/s
motor6Limit = 1000	#deg/s
omegaLimits = np.array([motor1to3Limit, motor1to3Limit, motor1to3Limit, motor4Limit, motor5Limit, motor6Limit])



global timeStamp
global sock

#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "192.168.65.220" #"192.254.82.154"
UDP_PORT = 1002

timeStamp = 0.05

def callback_omegas(data_floats):

	global timeStamp
	global sock

	frame = bytearray()

	# To denote sart of the UPD message	
	frame.append(ord('S'))
	frame.append(ord('S'))

	
	print('Omegas sent. timeStamp: ', timeStamp)
	# print(data_floats.data)


	# Add floats into the frame
	for i, f in enumerate(data_floats.data):
		# Pack the float into the frame
		f = np.clip(f, -omegaLimits[i], omegaLimits[i])
		ba = bytearray(struct.pack("f", f))
		for item in ba:
			frame.append(item)


	# Add the time stamp !!! This is not the correct time (it was just used to differentiate packets)
	ba = bytearray(struct.pack("f", timeStamp))
	for item in ba:
		frame.append(item)
	timeStamp = timeStamp + 0.05

	# To denote the termination of the UDP message
	frame.append(ord('T'))
	frame.append(ord('T'))


	rospy.loginfo(str(frame))
	sock.sendto(frame, (UDP_IP, UDP_PORT))


def listener():
	global sock
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	rospy.init_node(nodeName, anonymous=False)
	rospy.Subscriber(topicName, Floats, callback_omegas)
	rospy.spin()


if __name__ == '__main__':
	listener()
