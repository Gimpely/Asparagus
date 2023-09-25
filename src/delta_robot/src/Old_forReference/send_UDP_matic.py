#!/usr/bin/env  python3
import rospy
#from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Twist
import socket
import struct
import numpy


global timeStamp
global sock

#sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "192.168.65.220" #"192.254.82.154"
UDP_PORT = 1002

timeStamp = 0.05

def callback(twist):



	global timeStamp
	frame = bytearray()
	#rospy.loginfo(data)
	
    
	global sock
	#rospy.loginfo(rospy.get_caller_id()  + "I heard %s", data.data)
	#rospy.loginfo(rospy.get_caller_id() + "test: " + str(data.x))
	
	
	
	frame.append(ord('S'))
	frame.append(ord('S'))

#	frame.append(ord('8'))


	ba = bytearray(struct.pack("f", 3.14))
	for item in ba:
		frame.append(item)

	# ba = bytearray(struct.pack("f", twist.linear.x))
	# for item in ba:
	# 	frame.append(item)
	# ba = bytearray(struct.pack("f", twist.angular.z))
	# for item in ba:
	# 	frame.append(item)



	#ba = bytearray(struct.pack("f", twist.angular.x))
#	for item in ba:
#		frame.append(item)
	ba = bytearray(struct.pack("f", timeStamp))
	for item in ba:
		frame.append(item)
	timeStamp = timeStamp + 0.05
	#frame.append(ord('T'))
	#frame.append(ord('T'))

	rospy.loginfo(str(twist.linear.x))
	rospy.loginfo(str(twist.angular.z))
	rospy.loginfo(str(frame))
	sock.sendto(frame, (UDP_IP, UDP_PORT))
	#rate.sleep

def listener():
	global sock
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	rospy.init_node('prenos', anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
