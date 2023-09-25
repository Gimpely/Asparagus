#!/usr/bin/env python

import rospy
import socket
import struct
import binascii
import time
# from std_msgs.msg import String, Float64
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

import numpy as np


# User settings:

nodeName = 'UDP_recieve_alphas'
topicName = 'alphas_actual'
nodeRate = 100	# in Hz

displayAdditionalInfo = False
omegasLength = 6

# Debug:
lastTimestampPLC = 0


UDP_PORT = 1028
UDP_IP = ""

def recieve_packet(s):
	end = False
	packet = []
	while True:
		dataArr, addr = s.recvfrom(40)
		#data = dataArr[0]
		#rospy.loginfo(str(data))
		#rospy.loginfo(dataArr)
		#rospy.loginfo(len(dataArr))
		packet = list(dataArr)
		"""if str(data) == 'T' and end:
			return packet
		if str(data) == 'T':
			end = True
		else:
			end = False"""
		return packet

def decode_packet(packet):
	valid = False
	additionalInfo = None
	# rospy.loginfo('Function decode_packet: packet len: ' + str(len(packet)))

	if (str(packet[0]) == 'S' and str(packet[1]) == 'S' and str(packet[-1]) == 'T' and str(packet[-2]) == 'T'):
		valid = True
		# rospy.loginfo("packet is valid")
	if valid:
		# Get float values from the udp message to a numpy array:
		alphas = np.zeros((omegasLength), dtype=np.float32)
		i = 2
		for j in range(omegasLength):
			alphas[j] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
			i += 4


		if displayAdditionalInfo == True:
			# Read additional info
			nInfo = 2
			additionalInfo = np.zeros((nInfo), dtype=np.float32)

			for j in range(nInfo):
				additionalInfo[j] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
				i += 4


		# print("Recieved values (alphas):")
		# print(alphas)

		# # ----- Old way it was writen as: -----------------
		# odom_data = {}
		# i = 2
		# odom_data["vel_L"] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
		# i = i + 4
		# odom_data["vel_R"] = struct.unpack('>f', bytearray([packet[i+3], packet[i+2], packet[i+1], packet[i]]))[0]
		# i = i + 4
	
	return valid, alphas, additionalInfo
			

def talker():
	global lastTimestampPLC

	pub = rospy.Publisher(topicName, numpy_msg(Floats),queue_size=10)
	rospy.init_node(nodeName, anonymous=False)
	rate = rospy.Rate(nodeRate) # Hz

	# udp comm stuff
	global sock
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.bind((UDP_IP, UDP_PORT))


	while not rospy.is_shutdown():
		start_time = time.time()
		packet = recieve_packet(sock)
		valid, alphas, additionalInfo = decode_packet(packet)
		if valid:
			
			# rospy.loginfo('Alphas (UDP:)')
			# rospy.loginfo(alphas)

			displayPrint = 'Alphas (UDP): '
			for alpha in alphas:
				displayPrint += '	{:.02f}'.format(alpha)

			if displayAdditionalInfo == True:
				displayPrint += ' | '
				for aInfo in additionalInfo:
					displayPrint += '	{:.01f}'.format(aInfo)


				currentTimeStamp_PLC = additionalInfo[1]
				displayPrint += '  dT = {}'.format(currentTimeStamp_PLC - lastTimestampPLC)

				lastTimestampPLC = currentTimeStamp_PLC






			rospy.loginfo(displayPrint)

			pub.publish(alphas)




if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
