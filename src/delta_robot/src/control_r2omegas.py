#!/usr/bin/env  python3
from ast import If
from operator import truediv
from pickle import TRUE
import rospy

# Include msg to communicate with robot
from beckhoff_msgs.msg import CmdRobot
from beckhoff_msgs.msg import JointStateRobot

from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import time
import math
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped, Point

from supportingFunctions.inverseKinematics_Phi import deltaInverseKin
from supportingFunctions.deltaKinematics import deltaInvKin, deltaIForwardKin

class velocity_control:
	def __init__(self, Kp, Kd, maxOmegas, displayResult=False):
		self.Kp = np.asarray(Kp)
		self.Kd = np.asarray(Kd)
		self.maxOmegas = np.asarray(maxOmegas)
		self.displayResult = displayResult

		self.dt    = 0.0
		self.e_old = 0.0
		self.Timestamp_old = 0
		self.newData_flag = False
		self.dqd = np.zeros((5)) 
		self.prev_error = 0
		self.previous_error = np.array([0.0, 0.0, 0.0])
		self.integral_error = np.array([0.0, 0.0, 0.0])
		self.start_time = time.time()
		self.zacetni_cas = time.time()
		self.now = True
		self.stampOld = 0
		self.timeTimeforTime = time.time()
		self.dq_old = [0,0,0]
		self.deleteMe1 = []
		self.deleteMe2 = []
		self.deleteMe3 = []


		# Define node and topics
		# Names
		self.nodeName = 'control_r2omegas'
		# Subscribing to topic
		self.topicName_r_control = '/r_control' # Calculate trajectory of delta robot
		# Subscribing to topic
		self.topicName_delta_from_plc = '/robot/joint_state' # Angles from servo and DC motors -> We get them from beckhoff PLC over ADS communication
		# Publishing topics 
		self.topicName_delta_to_plc = '/robot/cmd' # Sending motor velocities to PLC

		rospy.init_node(self.nodeName, anonymous=False)

		self.qd_available = False
		
		rospy.Subscriber(self.topicName_r_control, Floats, self.callback_readReference, queue_size=1)
		rospy.Subscriber(self.topicName_delta_from_plc, JointStateRobot, self.callback_control, queue_size=1)
		rospy.Subscriber("/robot/qd", Floats, self.callback_read_qd, queue_size=1)
		rospy.Subscriber("/robot/close_gripper", Bool, self.callback_closeGripper, queue_size=1)
		self.close_gripper = False

		self.tcp_pub = rospy.Publisher("/detected_tcp_point", PointStamped, queue_size=1)
		self.pub = rospy.Publisher(self.topicName_delta_to_plc, CmdRobot, queue_size=1)

		
		
		
		self.stevec = 0

		######## added by sslajpah ########
		self.r_control = []
		self.qd = []
		self.start_time = time.time()
		###################################
		
	def pretty_np(self, preprintString, numpyArray, nDecimals=4):
		displayPrint = preprintString
		for number in numpyArray:
			displayPrint += '	{num:.0{prec}f}'.format(num=number, prec=nDecimals)
		return displayPrint

	def callback_closeGripper(self, data):
		self.close_gripper = data.data
		# print("close gripper clbk", self.close_gripper)

		#print("Close gripper = ", self.close_gripper)

	def callback_readReference(self, data):
		self.r_control = data.data

		# print('r_control = ', self.r_control)
		
		self.start_time = time.time()

		self.qd_available = False
		
		return 

	def callback_read_qd(self, data):
		self.qd = data.data

		self.start_time = time.time()

		# print(self.qd)

		self.qd_available = True

		return
	

	def callback_control(self, data):
		# Check if we received new data from PLC
		#print(f"timestamp {repr(data)}")
		qq = [0,0,0]



		# now = time.time()
		# # print("Time dif: ", (now - self.timeTimeforTime)*1000)
		# self.timeTimeforTime = now



		if data.Timestamp != self.Timestamp_old:
			self.newData_flag = True
		
		#print(data.Timestamp)
		

		flagVelocety = False

		try:
			len(self.r_control)!=0
			flagVelocety = True
		except: 
			pass

		if self.newData_flag and flagVelocety == True :
			self.newData_flag = False
			#dt = (data.Timestamp - self.Timestamp_old)
			#print("delta stevec: " + str(dt))
			self.Timestamp_old = data.Timestamp


			# if self.now:
			# 	self.stampOld = data.Timestamp
			# 	self.now = False
			# currStamp = data.Timestamp	
			# print("stam diff: ", currStamp-self.stampOld)
			# self.stampOld = currStamp
			

			# Read actual angles
			
			self.qq = [data.qq.j0, data.qq.j1, data.qq.j2, data.qq.j3, data.qq.j4]
			self.qq = np.asarray(self.qq, dtype=np.float32)



			qq = [data.qq.j0, data.qq.j1, data.qq.j2]
			qq = np.asarray(qq, dtype=np.float32)
			# rospy.logdebug("Control timestamp: %s, values: %s", currStamp, qq)
			# print(data)

			

			# print("read angels:", qq)



			# read actual velocity
			self.dq = [data.dq.j0, data.dq.j1, data.dq.j2, data.dq.j3, data.dq.j4]
			self.dq = np.asarray(self.dq, dtype=np.float32)




			# Desired axis velocity
			qd = np.zeros((5)) 
			# Desired axis velocity in radians
			omega = 3
			
			x_sin_test = 100*math.sin(time.time()*4)
			#print(x_sin_test)

			# print('r_control = ', self.r_control[:3])

			# if self.qd_available == False:
			# 	qd_radians, _ = deltaInverseKin(self.r_control[0], self.r_control[1], self.r_control[2], self.r_control[3])
			# 	#print('qd_radians = ', qd_radians)
			# 	#qd_radians, _ = deltaInverseKin(x_sin_test, 0, -750, self.r_control[3])
			# 	# Transform to degrees
			# 	qd[:3] = 180/np.pi*qd_radians
			# 	qd[3:] = self.r_control[3:]
			# else:
			# 	qd = self.qd


			# print("r_ctrl:",self.r_control[:3])
			# print("qd_available",self.qd_available )
			# print("qd",self.qd)
			
			####### added by sslajpah, 24. 11. 2022  #######
			if self.qd_available == False :
				if len(self.r_control)!=0:
					# print("r_cotrol",self.r_control)
					# qd_radians, _ = deltaInverseKin(self.r_control[0], self.r_control[1], self.r_control[2], self.r_control[3])
					qd_radians, eror = deltaInvKin(self.r_control[0], self.r_control[1], self.r_control[2])
			
					

					# print("Iz inv kin: ", qd_radians)

					#print('qd_radians = ', qd_radians)
					#qd_radians, _ = deltaInverseKin(x_sin_test, 0, -750, self.r_control[3])
					# Transform to degrees
					qd[:3] = 180/np.pi*qd_radians
					qd[3:] = self.r_control[3:]
					# print("Old: ",qd[:3])
					# print("New: ",np.degrees(qd_test))
			else:
				qd = self.qd
				if np.isnan(qd[0]) or np.isnan(qd[1]) or np.isnan(qd[2]):
					print("Dobil nan")
					qd = self.qq
			
			#####################################

			
			# print("qd = ", qd)
			# test PD control
			'''
			self.stevec = self.stevec + 1			
			if self.stevec < 5000:
				aref = 10
			else:
				aref = 30
				if self.stevec > 10000:
					self.stevec = 0

			qd[0] = aref #15+10*math.sin(time.time()*2)
			qd[1] = 5 #
			qd[2] = 5 #
			'''
			

			#self.testpub.publish(qd[0])

			
			#print('qd = ', qd)
			#print("desired poz = ", qd)

			# angular error - joint coordinates
			# print("qq", self.qq)
			
			# print("qq", self.qq[:3])
			# print("dq", self.dq[:3])
			# print("qd: ", qd[:3])
			# radijani = qd[:3] * np.pi/180
			# ref_poz = deltaIForwardKin(radijani)
			# print("ref_pozition: ",[ref_poz[0].item(), ref_poz[1].item(),ref_poz[2].item()])
			
			# curr_poz = deltaIForwardKin(self.qq[:3])
			# print("curr_pozition: ",[curr_poz[0].item(), curr_poz[1].item(),curr_poz[2].item()])
			# print("----------------------")

			


			
			dq = np.zeros((5), dtype=np.float32)
			###################### Control delta robot ######################
			
			e = qd - self.qq
			de = -self.dq
			cas_zdaj = time.time()
			dt = cas_zdaj - self.zacetni_cas

			dq = 5*e + self.Kd * de

			if np.abs(e[0])<0.05:
				dq[0] = 0
			if np.abs(e[1])<0.05:
				dq[1] = 0
			if np.abs(e[2])<0.05:
				dq[2] = 0
			
			
			# print("stamp", data.Timestamp)

			# print("read angels (self)", self.qq[:3])


			# print("qq error: ", e[:3])
			# curr_poz = deltaIForwardKin(self.qq[:3])
			# print("r_control", self.r_control[:3])
			# print("curr_position: ",[curr_poz[0].item(), curr_poz[1].item(),curr_poz[2].item()])




			


		
			
			################### Contorl for the gripper rotation #########################
			error = e[3]
			
			self.zacetni_cas = cas_zdaj
			Kpp = 1.84
			Kdd = 0.38
			Kii = 0.1
			self.prev_error += error * dt
			self.prev_error = max(-180, min(180,self.prev_error))
			

			dq[3] = Kpp*error + Kii*self.prev_error + Kdd* de[3]
			# if np.abs(dq[3]) < 2:
			# 	dq[3] = 2
			if np.abs(error) < 3:
				dq[3] = 0
			# print("E:", e[3])
			# print("E integral: ", self.prev_error)
			# print("po regulaciji: ", dq[3])

			######### Limit gripper opening ######
			# if dq[4]<-90:
			# 	dq[4]=-90
				


			dq = np.clip(dq, -self.maxOmegas, self.maxOmegas)
			dq = dq.astype(np.float32)
			
			# Save old error
			self.e_old = e
			# Save current time
			time_ms = time.time()
			# Calculate cycle time
			self.dt = time_ms - self.start_time

			#RobotCmd = np.zeros((6), dtype=np.float32)
			RobotCmd = CmdRobot()

			# Desired velocity
			self.dqd = dq

			
			
			# Write data to array
			RobotCmd.Timestamp = rospy.get_rostime()
			RobotCmd.dq.j0 = dq[0]
			RobotCmd.dq.j1 = dq[1]
			RobotCmd.dq.j2 = dq[2]
			RobotCmd.dq.j3 = dq[3]
			RobotCmd.dq.j4 = dq[4]
			# print(RobotCmd.dq.j4)
			# print(RobotCmd)
			
			if abs(RobotCmd.dq.j0) > 10 or abs(RobotCmd.dq.j1) > 10 or abs(RobotCmd.dq.j2) > 10 or abs(RobotCmd.dq.j3) > 10 :
				kkkk = 0
				pass 

			
			if self.close_gripper == True:
				RobotCmd.dq.j4 = 0
				# Open gripper
				RobotCmd.open_gripper  = 1
				RobotCmd.close_gripper = 0
			elif self.close_gripper == False:
				RobotCmd.dq.j4 = 0
				# Open gripper
				RobotCmd.open_gripper  = 0
				RobotCmd.close_gripper = 1
			else:
				RobotCmd.dq.j4 = dq[4]
				RobotCmd.open_gripper  = 0
				RobotCmd.close_gripper = 0

			# print(RobotCmd)

			# print('Open gripper = ', RobotCmd.open_gripper)
			# print('Close gripper = ', RobotCmd.close_gripper)

			#print('--------------------------')
			#print(data_to_send)
			#print(str((self.dt)*1000) + ' ms')
			#print('--------------------------')
			
			# print("na bckoff: ", dq[:3])
			
			# print("\n")
			# RobotCmd.dq.j0 = 0.0
			# RobotCmd.dq.j1 = 0.0
			# RobotCmd.dq.j2 = 0.0
			# RobotCmd.dq.j3 = 0.0
			# RobotCmd.dq.j4 = 0.0
			# print(RobotCmd.dq.j4)
			self.pub.publish(RobotCmd)
			# print(RobotCmd)
			


			tcp = self.r_control[:3]
		 # Publish the TCP  point for visualization
			tcp_msg = PointStamped() 
			tcp_msg.header.stamp = rospy.Time.now()
			tcp_msg.header.frame_id = "robot"
			tcp_msg.point = Point(*tcp)
			self.tcp_pub.publish(tcp_msg)


			# Display data
			if self.displayResult:

				print(self.pretty_np('alphas ref:', qd))
				print(self.pretty_np('alphas act:', self.qq))
				print('-	-	-	-	-	-	-')
				print(self.pretty_np('\tP:', self.Kp*e, 4))
				print(self.pretty_np('\tD:', self.Kd*de, 4))
				print(self.pretty_np('omegas_ctrl:', dq))
	
	def topics(self):

		self.pub = rospy.Publisher(self.topicName_delta_to_plc, CmdRobot, queue_size=1)

		self.testpub = rospy.Publisher("test_data", Float32, queue_size=1)

		rospy.spin()


if __name__ == "__main__":
	
	
	Kp_delta = 5
	Kp = [Kp_delta,Kp_delta,Kp_delta, 2.5, 0.5]
	#rospy.set_param('/robot_Kp', [5,5,5,2,3])
	Kd_delta = 0.05
	Kd_DC = 0.05
	Kd = [Kd_delta, Kd_delta, Kd_delta, 0, 0]
	#rospy.set_param('/robot_Kd', [0.005,0.005,0.005,0.4,0.4])

	maxOmega = [190,190,190,250,250]
	# maxOmega = [210,210,210,250,250]

	displayResults = False

	control = velocity_control(Kp, Kd, maxOmega, displayResults)
	rospy.spin()
	# control.topics()








