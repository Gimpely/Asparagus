#!/usr/bin/env python3
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Pose2D
from beckhoff_msgs.msg import JointStateRobot
import numpy as np
import math
import time
from beckhoff_msgs.msg import CmdTracks
from std_msgs.msg import Bool


from supportingFunctions.inverseKinematics_Phi import deltaInverseKin
from supportingFunctions.deltaKinematics import deltaInvKin, deltaIForwardKin
from supportingFunctions.minJerkInterpolator import minJerkInterpolator
from supportingFunctions.minJerkContinuousInterpolation import minJerkContinuousInterpolation

class motion_planning:
	def __init__(self, xy_limits, z_start=0.15):
		# Read parameters 
		self.x_min_limit = xy_limits[0]
		self.x_max_limit = xy_limits[1]
		self.y_min_limit = xy_limits[2]
		self.y_max_limit = xy_limits[3]

		self.z_start = z_start
		
		rospy.init_node("motion_planning")
		
		# Tracks cmd
		self.cmdTracks = CmdTracks()

		self.pub = rospy.Publisher('/r_control', numpy_msg(Floats),queue_size=1)
		self.pub_cmd_qd = rospy.Publisher('/robot/qd', numpy_msg(Floats),queue_size=1)
		self.pub_cmdTrack = rospy.Publisher('/tracks/cmd', CmdTracks, queue_size=1)
		self.pub_closeGripper = rospy.Publisher('/robot/close_gripper', Bool, queue_size=1)

		self.timeflag = 0
		self.print_step = 0
		self.pub_idx = 0

		self.first_flag = False

		self.CloseGripper = False



		self.track_pose = Pose2D()

		self.cold_start = True

		self.picked_asparagus = np.array([[0, 0, 0 ,0]])
		self.first_asparagus_picked = False
		
		# State machine for path planning
		self.step_path = 0
		self.path_calculated = False
		# State machine for motion control
		self.gripper_open_poz  = -120
		self.gripper_close_poz = 0
		self.home_poz = [-0.2, 0, self.z_start, 0, self.gripper_open_poz]
		self.set_poz = np.copy(self.home_poz)
		
		self.step_motion = 0
		# Time gripper needs to close
		self.gripper_time = 1
		# Set place position for asparagus
		self.place_poz = [[-0.15, 0.22, 0.15, 0, self.gripper_close_poz], [-0.15, -0.22, 0.15, 0, self.gripper_close_poz]]
		self.new_path_request = True
		self.z_offset = -900

		self.next_point = self.convert_point_to_robot_cs(self.set_poz, self.z_offset)

		self.count_time = 0
		self.start_time = 0

		self.stop_platform = False

		# Interpolation
		dT_ms = int(1000/1000)

		# initialize Interpolator:
		self.maxVel = 300
		self.rotVel = 50
		self.maxSpeed = np.array([self.maxVel, self.maxVel, self.maxVel, self.rotVel*5, self.rotVel*5])
		self.interpolator = minJerkInterpolator(maxSpeed=self.maxSpeed, dT=dT_ms*0.001, printAll=False)
		maxJointSpeed = [190, 190, 190, 250, 250]
		self.continuous_interpolator = minJerkContinuousInterpolation(maxSpeed=maxJointSpeed, dT=dT_ms*0.001, printAll=False)

		self.robot_in_poz = False
		self.robot_in_poz_inter = False

		# Subscribe to asparagus locations in global cs
		rospy.Subscriber("/aspragus_locations", numpy_msg(Floats), self.callback_aspragus_locations, queue_size=10)
		# Subcribe to track robot pose
		rospy.Subscriber("/tracks/pose", Pose2D, self.callback_track_pose)
		# Read angles of delta robot motors
		rospy.Subscriber('/robot/joint_state', JointStateRobot, self.callback_robot_angles)

		self.cas_zacetka = time.time() # zacetek pobiranja
		self.stevilo = 1 # stevilo pobranih

		


	
	def callback_track_pose(self, data):
		#print("ok")
		self.track_pose = data

		# Start robot motion
		# print("Start robot motion")
		self.robot_motion()

		#qd_radians = np.zeros(5)
		self.next_point = self.convert_point_to_robot_cs(self.set_poz, self.z_offset)
		if self.first_flag == False:
			self.next_point_old = self.next_point
			self.next_point_old[2] = 0
			self.distance = self.next_point_old
			for idx in range(len(self.distance)):
				self.distance[idx] = 0
			self.first_flag = True
		# print("inv 0")
		# qd_radians, eror = deltaInvKin(self.next_point[0], self.next_point[1], self.next_point[2])
		# if any(eror):
		# 		print("point 1", self.next_point[:3])
		# 		rospy.sleep(3)
		
		qd_radians, _ = deltaInverseKin(self.next_point[0], self.next_point[1], self.next_point[2], self.next_point[3])
		desired_angle_deg = 180 / np.pi *(qd_radians)
		
		#print("self.next_point  = ", self.next_point)
		#print("qd  = ", desired_angle_deg)

		
		if np.any(self.next_point_old != self.next_point):
			self.distance = np.abs(self.next_point - self.next_point_old)
			
			if np.max(self.distance) > 5:
				self.interpolator.addPoint(self.next_point)

			self.next_point_old = self.next_point 

			#self.interpolator.printPoints() 

		status, r_control = self.interpolator.run(self.robot_in_poz_inter)
		self.robot_in_poz_inter = False
		#status = 0
		if status > 0 and self.pub_idx == 1:

			self.pub_idx = 0
			r_control = r_control.astype(dtype=np.float32)
			#print("r_control = ", r_control)
			# print("inv 1")
			# qd_radians, eror= deltaInvKin(r_control[0], r_control[1], r_control[2])
			# if any(eror):
			# 	print("point 2", r_control[:3])
			# 	rospy.sleep(3)
			qd_radians, _ = deltaInverseKin(r_control[0], r_control[1], r_control[2], r_control[3])
			desired_angle_deg = 180 / np.pi *(qd_radians)
			#print("qd 2  = ", desired_angle_deg)

			self.next_point = self.next_point.astype(dtype=np.float32)
			
			#if np.max(self.distance) > 5:
				#self.pub.publish(r_control)
			#else:
				#self.pub.publish(self.next_point)

		self.pub_idx = self.pub_idx + 1

		if self.count_time == 0:
			self.start_time = time.time()
		if self.count_time == 1:
			#print(time.time() - self.start_time)
			self.count_time = 0

		self.count_time = self.count_time + 1
		


	def callback_robot_angles(self, data):
		self.qq = [data.qq.j0, data.qq.j1, data.qq.j2, data.qq.j3, data.qq.j4]
		self.qq = np.asarray(self.qq, dtype=np.float32)

		# test interpolation
		if self.first_flag:

			t = time.time()
			current_poz = self.qq[:3]
			

			qd_radians, _ = deltaInverseKin(self.next_point[0], self.next_point[1], self.next_point[2], self.next_point[3])
			
			# qd_radians, eror = deltaInvKin(self.next_point[0], self.next_point[1], self.next_point[2])
			# if any(eror):
			# 	print("point 3", self.next_point[:3])
			# 	rospy.sleep(3)
			desired_angle_deg = 180 / np.pi *(qd_radians)
			ref_position = desired_angle_deg[:3]

			#print("ref_position = ", ref_position)
			#print("current_poz = ", current_poz)

			tf = self.continuous_interpolator.dynamic_final_time(t, ref_position, current_poz)
			#print("tf = ", tf)
			xd = self.continuous_interpolator.minJerkInterpolation(t, tf, current_poz, ref_position)

			#print("x = ", current_poz)
			#print("xd = ", xd)

			# Publish desired angles to PD regulator
			xd_send = np.zeros(5)
			xd_send[:3] = xd
			xd_send[3] = self.next_point[3]
			xd_send[4] = self.next_point[4]

			xd_send = xd_send.astype(dtype=np.float32)

			self.pub_cmd_qd.publish(xd_send)

			#print("xd_send = ", xd_send)


	def callback_aspragus_locations(self, data):
		self.asparagus_locations = data.data
		self.asparagus_locations = np.asarray(self.asparagus_locations)
		

		# Transform data from 1D array to 2D array
		
		self.asparagus_locations = np.reshape(self.asparagus_locations, (-1,4))
		#print(self.asparagus_locations)
		# Filter points that are inside work space of a delta robot
		if len(self.asparagus_locations) > 0:
			self.locations_out_of_range(self.asparagus_locations)
			#self.cleaned_locations = self.asparagus_locations



		# print("got location: ", self.asparagus_locations)



		# Plan path for robot	
		if self.new_path_request:
			# print("new request za path", self.new_path_request)
			self.path_planning(min_dist=0.03, pretarget_dist=0.15, calculation_step=0.01)

	def locations_out_of_range(self, asparagus_locations):
		# Transform locations from global cs to robot cs and delete points that robot can't reach (out of range)
		delete_idx = []
		self.asparagus_locations_robot_cs = np.copy(asparagus_locations)
		for idx, i in enumerate(asparagus_locations):
			x_trans = self.track_pose.x * math.cos(self.track_pose.theta)
			y_trans = self.track_pose.y * math.sin(self.track_pose.theta)

			# Transform points from global cs to robot cs
			x_robot_cs = i[0] - x_trans
			y_robot_cs = i[1] - y_trans

			self.asparagus_locations_robot_cs[idx][0] = x_robot_cs
			self.asparagus_locations_robot_cs[idx][1] = y_robot_cs

			# Check for limits in x direction
			if x_robot_cs < self.x_min_limit or x_robot_cs > self.x_max_limit:
				# Check for limits in y direction
				if y_robot_cs < self.y_min_limit or y_robot_cs > self.y_max_limit:
					delete_idx = np.append(delete_idx, idx)

		# Delete points that are out of range
		if len(delete_idx) > 0:
			self.cleaned_locations = np.delete(self.asparagus_locations, delete_idx)
			self.cleaned_locations_robot_cs = np.delete(self.asparagus_locations_robot_cs, delete_idx)
		else:
			self.cleaned_locations = self.asparagus_locations
			self.cleaned_locations_robot_cs = self.asparagus_locations_robot_cs

		print("cleaned", self.cleaned_locations_robot_cs)
		'''
		# Check if picked asparagus are out of range
		if self.first_asparagus_picked:
			#self.picked_asparagus = np.reshape(self.picked_asparagus, (-1,4))
			delete_idx = []
			if len(self.picked_asparagus) > 0:
				#print(len(self.picked_asparagus))
				#print('test = ', self.picked_asparagus)
				for idx, i in enumerate(self.picked_asparagus):
					x_trans = self.track_pose.x * math.cos(self.track_pose.theta)
					y_trans = self.track_pose.y * math.sin(self.track_pose.theta)

					# Transform points from global cs to robot cs
					x_robot_cs = i[0] - x_trans
					y_robot_cs = i[1] - y_trans

					# Check for limits in x direction
					if x_robot_cs < self.x_min_limit or x_robot_cs > self.x_max_limit:
						# Check for limits in y direction
						if y_robot_cs < self.y_min_limit or y_robot_cs > self.y_max_limit:
							delete_idx = np.append(delete_idx, idx)

			# Delete points that are out of range
			#print(delete_idx)
			if len(delete_idx) > 0:
				if len(delete_idx)==1:
					delete_idx = int(delete_idx)

				if len(self.picked_asparagus) > 1:
					self.picked_asparagus = np.delete(self.picked_asparagus, delete_idx)
				else:
					self.picked_asparagus = np.array([[1000, 1000, 1000 ,0]])
		'''


	def robot_in_position(self, desired_angle, actual_angle, angle_error):
		# Check if delta robot reached desired position
		# Transform desired angle to deegres
		desired_angle_deg = 180 / np.pi *(desired_angle)
		# Calculate error
		e = desired_angle_deg[:3] - actual_angle[:3]

		if self.print_step == 10:
			self.print_step = 0
			#print('Desired poz', desired_angle_deg)
			#print('Actual poz', actual_angle)
			#print("error = ", e)
		self.print_step = self.print_step + 1
		
		# Check if each joint reached desired angle
		poz_reached = np.full((len(e)), False)
		for idx,i in enumerate(e):
			if abs(i) < angle_error[idx]:
				poz_reached[idx] = True
			else:
				poz_reached[idx] = False

		# Check if all joints reached desired position
		#robot_in_poz = np.all(poz_reached == True)
		#print(robot_in_poz)

		return np.all(poz_reached == True)

	def convert_point_to_robot_cs(self, point, z_offset):
		point_robot = np.zeros(len(point))
		
		temp= np.multiply(point[:3],1000)
		point_robot[:3] = temp #point[:3] * 1000
		point_robot[2] = point[2]*1000 + z_offset
		point_robot[3:] = point[3:]

		return point_robot

	def path_planning(self, min_dist, pretarget_dist, calculation_step):
		'''

		Plan delta robot path, based on number of asparagus detected and their locations

		'''
		print("current step: ",self.step_path)
		
		# Check how many asparagus are detected
		self.num_of_asparagus = len(self.cleaned_locations)
		# print("Num. of deteced: ",self.num_of_asparagus )
		#print('Len picked = ', len(self.picked_asparagus))
		#print('Len close = ', len(self.asparagus_too_close))
		#print('Len cleaned_locations = ', self.num_of_asparagus)
		#print('step path = ', self.step_path)

		

		#if self.num_of_asparagus < 1:
		#	self.step_path = 0
			#return

		if self.step_path == 0:
			
			# init parameters
			self.pick_idx = 0
			self.ready_for_pick = False
			self.grow_poz = np.zeros(3)
			self.asparagus_too_close = np.full(self.num_of_asparagus, False)
			# Angle at which we are trying to generate picking trajectory
			self.try_angle = 0
			self.gripper_poz = np.zeros(4)
			self.picked_asparagus_flag_arr = np.full(self.num_of_asparagus, False)

			

			if self.num_of_asparagus > 0:
				self.step_path = 0.5
				

		elif self.step_path == 0.5:
			
			# Check which asparagus were already harvested
			if len(self.picked_asparagus) > 0:
				self.pick_dist_array = np.zeros((len(self.picked_asparagus), self.num_of_asparagus))
				for idx, i in enumerate(self.picked_asparagus):
					for idx2, j in enumerate(self.cleaned_locations):
						dist = np.linalg.norm(j[:3] - i[:3])
						self.pick_dist_array[idx, idx2] = dist

				# Find minimum distances
				for i in self.pick_dist_array:
					min_dist_measured = min(i)
					if min_dist_measured < 0.02:
						min_idx =np.argmin(i)
						self.picked_asparagus_flag_arr[min_idx] = True

			#print(self.picked_asparagus_flag_arr)

			self.step_path = 1

		elif self.step_path == 1:
			
			
			
			if len(self.cleaned_locations) != len(self.asparagus_too_close):
				
				temp = np.full(self.num_of_asparagus, False)
				size_of_old = len(self.asparagus_too_close)
				if size_of_old >= self.num_of_asparagus:
					
					#temp = np.full(len(self.asparagus_too_close), False)
					temp[:self.num_of_asparagus] = self.asparagus_too_close[-self.num_of_asparagus:]
					
				else:
					
					#temp = np.full(self.num_of_asparagus, False)
					temp[:size_of_old] = self.asparagus_too_close
					

				

				
				#if size_of_old < self.num_of_asparagus:
				#	temp[:size_of_old] = self.asparagus_too_close
				#else:
				#	temp[:self.num_of_asparagus] = self.asparagus_too_close
				self.asparagus_too_close = temp
				
			
			if len(self.cleaned_locations) != len(self.picked_asparagus_flag_arr):
				temp = np.full(self.num_of_asparagus, False)

				size_of_old = len(self.picked_asparagus_flag_arr)
				if size_of_old >= self.num_of_asparagus:
					temp[:self.num_of_asparagus] = self.picked_asparagus_flag_arr[-self.num_of_asparagus:]
				else:
					#temp = np.full(self.num_of_asparagus, False)
					temp[:size_of_old] = self.picked_asparagus_flag_arr
					
				
				
				
				self.picked_asparagus_flag_arr = temp
			

			# Check asparagus height
			#print('Cleaned_location = ', self.cleaned_locations)
			for idx, i in enumerate(self.cleaned_locations):
				# 4th element in array represents if asparagus is ready for harvesting (is high enough)
				# Check if asparagus are too close and if asparagus was already picked
				if i[3] and self.asparagus_too_close[idx] == False and self.picked_asparagus_flag_arr[idx] == False:
					self.pick_idx = idx
					print('Pick_idx = ', self.pick_idx)
					self.ready_for_pick = True
					self.grow_poz = i[:3]

					# Go to next step 
					self.step_path = 2

					break

			if self.ready_for_pick == False:
				self.step_path = 0
		 
		elif self.step_path == 2:
			
			# Check if any asparagus grows nearby
			# Calculate distances to all other asparagus
			
			self.dist_array = np.zeros(self.num_of_asparagus)
			for idx, i in enumerate(self.cleaned_locations):
				if idx != self.pick_idx:
					dist = np.linalg.norm(self.grow_poz - i[:3])
					self.dist_array[idx] = dist
				else:
					self.dist_array[idx] = 0

			self.step_path = 3

		elif self.step_path == 3:
			
			# Check if distance is smaller than minimum distance based on the size of the gripper
			for idx, i in enumerate(self.dist_array):
				if i != 0 and i < min_dist:
					# Picking not posible
					self.asparagus_too_close[idx] = True
					self.asparagus_too_close[self.pick_idx] = True
					
					# Check if we can pick other asparagus
					self.step_path = 1

					break
				else:
					self.step_path = 4
		
		elif self.step_path == 4:
			
			# Check if there is any obstacle (small aspragus, tracks) inside larger radius
			
			# Try picking from behind at 0 degree angle
			# Check if there is enough space behind asparagus, else try different direction
			# We already checked at pick position if there is space for gripper, now we go in -x direction
			self.grow_poz_robot_cs = self.cleaned_locations_robot_cs[self.pick_idx][:3]
			# Current gripper poz
			self.gripper_poz[:3] = self.grow_poz_robot_cs[:3]
			self.gripper_poz[3]  = self.try_angle

			self.obsticle_on_path = False
			self.gripper_dist_array = np.zeros(self.num_of_asparagus)

			
			current_pretarget_dist = np.linalg.norm(self.grow_poz_robot_cs - self.gripper_poz[:3])
			while current_pretarget_dist <= pretarget_dist:
				# Calculate distance between gripper and other asparagus and tracks
				for idx, i in enumerate(self.cleaned_locations_robot_cs):
					if idx != self.pick_idx:
						dist = np.linalg.norm(self.gripper_poz[3] - i[:3])
						self.gripper_dist_array[idx] = dist
					else:
						self.gripper_dist_array[idx] = 0
			
				if self.gripper_poz[1] >= 0:
					self.track_dist = abs(self.y_max_limit - self.gripper_poz[1])
				else:
					self.track_dist = abs(self.y_min_limit - self.gripper_poz[1])

				# Calculate distance to -x limit
				self.dist_to_x_limit = abs(self.x_min_limit - self.gripper_poz[0])

				# Check if all dist are larger than min dist
				if self.num_of_asparagus > 1:
					for i in self.gripper_dist_array:
						if i > min_dist and i != 0:
							self.dist_to_asparagus_ok = True
						elif i <= min_dist and i != 0:
							self.dist_to_asparagus_ok = False
							break
				else:
					self.dist_to_asparagus_ok = True

				if self.track_dist > min_dist:
					self.dist_tracks_ok = True
				else:
					self.dist_tracks_ok = False

				if self.dist_to_x_limit > min_dist:
					self.dist_x_ok = True
				else:
					self.dist_x_ok = False

				# Check if all conditions are true
				#print('self.dist_to_asparagus_ok = ', self.dist_to_asparagus_ok)
				#print('self.dist_tracks_ok = ', self.dist_tracks_ok)
				#print('self.dist_x_ok = ', self.dist_x_ok)
				#self.dist_to_asparagus_ok = True
				if self.dist_to_asparagus_ok and self.dist_tracks_ok and self.dist_x_ok:
					# go to next point
					self.gripper_poz[0] = self.gripper_poz[0] - calculation_step * math.cos(math.radians(self.try_angle))
					self.gripper_poz[1] = self.gripper_poz[1] - calculation_step * math.sin(math.radians(self.try_angle))
					current_pretarget_dist = np.linalg.norm(self.grow_poz_robot_cs - self.gripper_poz[:3])
				else:
					self.obsticle_on_path = True
					break
				

			if self.obsticle_on_path == False:
				# Add points to path
				# Start poz is with z up
				angle_rad = math.radians(self.try_angle)
				start_point = [self.gripper_poz[0], self.gripper_poz[1], self.z_start, angle_rad]
				x_avg = (self.gripper_poz[0] + self.grow_poz_robot_cs[0]) / 2
				y_avg = (self.gripper_poz[1] + self.grow_poz_robot_cs[1]) / 2
				intermedian_point = [x_avg, y_avg, self.grow_poz_robot_cs[2], angle_rad]
				pick_point = [self.grow_poz_robot_cs[0], self.grow_poz_robot_cs[1], self.grow_poz_robot_cs[2], angle_rad]
				
				self.path = [start_point, intermedian_point, pick_point]

				self.step_path = 5
				
			else:
				self.path = []
			#print('path = ', self.path)

	
		elif self.step_path == 5:
			
			# Transform points back to global cs
			self.path_global_cs = np.copy(self.path)
			for idx, i in enumerate(self.path):
				x_trans = self.track_pose.x * math.cos(self.track_pose.theta)
				y_trans = self.track_pose.y * math.sin(self.track_pose.theta)

				# Transform points from global cs to robot cs
				self.path_global_cs[idx][0] = i[0] + x_trans
				self.path_global_cs[idx][1] = i[1] + y_trans

			self.step_path = 6
			
			#print('global path = ', self.path_global_cs)

		elif self.step_path == 6:
			
			self.new_path_request = False
			self.path_calculated = True

			self.step_path = 0.5
	
	def robot_motion(self):

		
		#print('step_motion = ',self.step_motion)
		#print('Path calculated = ', self.path_calculated)
		#print('New path request = ', self.new_path_request)

		#print("robot in poz = ", self.robot_in_poz)

		
		# Calculate desired angles in robot axis
		angle_error = [1, 1, 1, 1, 1]			

		if self.step_motion == 0:
			
			self.robot_in_poz = False
			self.path_idx = 0
			# Check if path was already calculated, else go to home position
			if self.path_calculated:
				
				self.step_motion = 2
				self.timeflag = time.time()
			else:
				self.set_poz = self.home_poz
				self.step_motion = 1


		elif self.step_motion == 1:
			
			# Check if we reached home position
			qd_radians = np.zeros(5)
			temp = self.next_point #self.convert_point_to_robot_cs(self.set_poz, self.z_offset)
			# print("inv 3")

			# qd_radians[:3], eror = deltaInvKin(temp[0], temp[1], temp[2])
			# if any(eror):
			# 	print("point 4", temp[:3])
			# 	rospy.sleep(3)

			qd_radians[:3], _ = deltaInverseKin(temp[0], temp[1], temp[2], temp[3])

			angle_error = [0.5, 0.5, 0.5, 1, 1]
			self.robot_in_poz = self.robot_in_position(qd_radians, self.qq, angle_error)
			
			# Open gripper
			self.CloseGripper = False
			if self.robot_in_poz:
				print("in home")
				print(str(self.robot_in_poz) + " " + str(self.step_motion))
				timediff = time.time() - self.timeflag 
				print("Cas izvajanja: " + str(timediff))
				self.timeflag = time.time()
				print("step_path = ", self.step_path)
				print("")

				self.robot_in_poz = False
				self.robot_in_poz_inter = True
				self.step_motion = 2
			# print("--------")

		elif self.step_motion == 2:
			# print("Debug, motion 2:", self.step_motion)
			# print("caka na path", self.path_calculated)
			# Wait for path to be calculated
			if self.path_calculated:
				# Stop platform motion
				self.cmdTracks.stop_tracks  = True
				self.cmdTracks.start_tracks = False

				# Publish cmd
				#self.pub_cmdTrack.publish(self.cmdTracks)

				print(str(self.robot_in_poz) + " " + str(self.step_motion))
				timediff = time.time() - self.timeflag 
				print("Cas izvajanja: " + str(timediff))
				self.timeflag = time.time()
				print("step_path = ", self.step_path)
				print("")

				rospy.loginfo("POBERI " + str(self.stevilo))
				self.cas_zacetka = time.time()

				self.step_motion = 3
			# print("--------")

		elif self.step_motion == 3:

			# Transform path points to robot cs
			x_trans = self.track_pose.x * math.cos(self.track_pose.theta)
			y_trans = self.track_pose.y * math.sin(self.track_pose.theta)

			# Transform points from global cs to robot cs
			x_robot_cs = self.path_global_cs[self.path_idx][0] - x_trans
			y_robot_cs = self.path_global_cs[self.path_idx][1] - y_trans
			
			# Send robot to path point
			self.set_poz = [x_robot_cs, y_robot_cs, self.path_global_cs[self.path_idx][2], self.path_global_cs[self.path_idx][3], self.gripper_open_poz]

			qd_radians = np.zeros(5)
			temp = self.next_point #self.convert_point_to_robot_cs(self.set_poz, self.z_offset)
			# print("inv 4")

			# qd_radians[:3], eror = deltaInvKin(temp[0], temp[1], temp[2])
			# if any(eror):
			# 	print("point 5", temp[:3])
			# 	rospy.sleep(3)

			qd_radians[:3], _ = deltaInverseKin(temp[0], temp[1], temp[2], temp[3])

			# angle_error = [3, 3, 3, 1, 1] #original
			angle_error = [5, 5, 5, 2, 2]
			self.robot_in_poz = self.robot_in_position(qd_radians, self.qq, angle_error)

			if self.robot_in_poz:
				print(str(self.robot_in_poz) + " " + str(self.step_motion))
				timediff = time.time() - self.timeflag 
				print("Cas izvajanja: " + str(timediff))
				self.timeflag = time.time()
				print("step_path = ", self.step_path)
				print("")

				print(self.robot_in_poz)
				self.robot_in_poz = False

				self.robot_in_poz_inter = True

				self.path_idx = self.path_idx + 1

				self.step_motion = 4

		elif self.step_motion == 4:
		
			# Check if we are at the last point (pick point)
			if self.path_idx >= len(self.path_global_cs):
				self.CloseGripper = True
				# Start time measurement
				self.start_gripper_closing = time.time()

				print(str(self.robot_in_poz) + " " + str(self.step_motion))
				timediff = time.time() - self.timeflag 
				print("Cas izvajanja: " + str(timediff))
				self.timeflag = time.time()
				print("step_path = ", self.step_path)
				print("")

				self.step_motion = 5
			else:
				self.step_motion = 3

		elif self.step_motion == 5:
		
			# Hold pick position and close the gripper
			# Transform first path point to robot cs
			x_trans = self.track_pose.x * math.cos(self.track_pose.theta)
			y_trans = self.track_pose.y * math.sin(self.track_pose.theta)

			# Transform points from global cs to robot cs
			x_robot_cs = self.path_global_cs[-1][0] - x_trans
			y_robot_cs = self.path_global_cs[-1][1] - y_trans
			
			# Send robot to path point
			self.set_poz = [x_robot_cs, y_robot_cs, self.path_global_cs[-1][2], self.path_global_cs[-1][3], self.gripper_close_poz]

			qd_radians = np.zeros(5)
			temp = self.next_point #self.convert_point_to_robot_cs(self.set_poz, self.z_offset)
			# print("inv 5")

			# qd_radians[:3], eror= deltaInvKin(temp[0], temp[1], temp[2])
			# if any(eror):
			# 	print("point 6", temp[:3])
			# 	rospy.sleep(3)

			qd_radians[:3], _ = deltaInverseKin(temp[0], temp[1], temp[2], temp[3])
			
			elapsed_time = time.time() - self.start_gripper_closing

			if elapsed_time > self.gripper_time:
				print(str(self.robot_in_poz) + " " + str(self.step_motion))
				timediff = time.time() - self.timeflag 
				print("Cas izvajanja: " + str(timediff))
				self.timeflag = time.time()
				print("step_path = ", self.step_path)
				print("")

				self.cmdTracks.stop_tracks  = False
				self.cmdTracks.start_tracks = True

				# Publish cmd
				#self.pub_cmdTrack.publish(self.cmdTracks)

				self.step_motion = 6
		
		elif self.step_motion == 6:
		
			# Send gripper up
			# Transform first path point to robot cs
			x_trans = self.track_pose.x * math.cos(self.track_pose.theta)
			y_trans = self.track_pose.y * math.sin(self.track_pose.theta)

			# Transform points from global cs to robot cs
			x_robot_cs = self.path_global_cs[-1][0] - x_trans
			y_robot_cs = self.path_global_cs[-1][1] - y_trans
			
			# Send robot to path point
			self.set_poz = [x_robot_cs, y_robot_cs, self.z_start, self.path_global_cs[-1][3], self.gripper_close_poz]

			qd_radians = np.zeros(5)
			temp = self.next_point #self.convert_point_to_robot_cs(self.set_poz, self.z_offset)
			# print("inv 6")

			# qd_radians[:3], eror= deltaInvKin(temp[0], temp[1], temp[2])
			# if any(eror):
			# 	print("point 7", temp[:3])
			# 	rospy.sleep(3)

			qd_radians[:3], _ = deltaInverseKin(temp[0], temp[1], temp[2], temp[3])

			# angle_error = [3, 3, 3, 1, 1]
			angle_error = [10, 10, 10, 10, 10]
			self.robot_in_poz = self.robot_in_position(qd_radians, self.qq, angle_error)

			if self.robot_in_poz:
				print(str(self.robot_in_poz) + " " + str(self.step_motion))
				timediff = time.time() - self.timeflag 
				print("Cas izvajanja: " + str(timediff))
				self.timeflag = time.time()
				print("step_path = ", self.step_path)
				print("")

				self.robot_in_poz = False

				self.robot_in_poz_inter = True

				# Save position of picked asparagus
				
				if self.first_asparagus_picked==False:
					self.picked_asparagus = np.array([[self.path_global_cs[-1][0], self.path_global_cs[-1][1], self.path_global_cs[-1][2], self.path_global_cs[-1][3]]])
				else:
					self.picked_asparagus = np.append(self.picked_asparagus, [[self.path_global_cs[-1][0], self.path_global_cs[-1][1], self.path_global_cs[-1][2], self.path_global_cs[-1][3]]], axis=0)

				self.first_asparagus_picked = True

				self.path_calculated = False
				self.new_path_request = True

				self.step_motion = 7

		elif self.step_motion == 7:
		
			# Go to place position
			# Check y coordinate of the gripper -> go to closer storage
			if self.set_poz[1] >= 0:
				# Go to left storage
				self.storage_idx = 0
			else:
				# Go to right storage
				self.storage_idx = 1

			self.set_poz = self.place_poz[self.storage_idx]
			
			qd_radians = np.zeros(5)
			temp = self.next_point #self.convert_point_to_robot_cs(self.set_poz, self.z_offset)
			# print("inv 6")

			qd_radians[:3], _ = deltaInverseKin(temp[0], temp[1], temp[2], temp[3])

			# qd_radians[:3],eror= deltaInvKin(temp[0], temp[1], temp[2])
			# if any(eror):
			# 	print("point 8", temp[:3])
			# 	rospy.sleep(3)

			angle_error = [1, 1, 1, 1, 1]
			self.robot_in_poz = self.robot_in_position(qd_radians, self.qq, angle_error)

			if self.robot_in_poz:
				print(str(self.robot_in_poz) + " " + str(self.step_motion))
				timediff = time.time() - self.timeflag 
				print("Cas izvajanja: " + str(timediff))
				self.timeflag = time.time()
				print("step_path = ", self.step_path)
				print("")

				self.robot_in_poz = False

				self.robot_in_poz_inter = True

				# Open gripper
				self.set_poz[4] = self.gripper_open_poz
				self.CloseGripper = False
				self.start_gripper_opening = time.time()



				self.step_motion = 8

		elif self.step_motion == 8:
			
			# Open gripper
			elapsed_time = time.time() - self.start_gripper_opening

			if elapsed_time > self.gripper_time:
				print(str(self.robot_in_poz) + " " + str(self.step_motion))
				timediff = time.time() - self.timeflag 
				print("Cas izvajanja: " + str(timediff))
				self.timeflag = time.time()
				print("step_path = ", self.step_path)
				print("")

				rospy.loginfo("ODLOZI "+ str(self.stevilo))
				casPobiranja = time.time()-self.cas_zacetka
				rospy.loginfo("Cas pobiranja: " + str(casPobiranja))
				self.stevilo += 1

				self.step_motion = 9

		elif self.step_motion == 9:
		
			# Check if next path is available, else go to home position
			self.step_motion = 0

		# Publish gripper cmd
		self.pub_closeGripper.publish(self.CloseGripper)


	#def check_asparagus_position


if __name__ == "__main__":
	# Call class
	# xy_limits = [-0.3, 0.3, -0.3, 0.3]
	xy_limits = [-0.2, 0.3, -0.3, 0.3]

	path_planning = motion_planning(xy_limits=xy_limits)
	rospy.spin()
	
	#path_planning = motion_planning(xy_limits=xy_limits)

	#rate = rospy.Rate(500)

	#while not rospy.is_shutdown():
	#	path_planning = motion_planning(xy_limits=xy_limits)
	#	rate.sleep()