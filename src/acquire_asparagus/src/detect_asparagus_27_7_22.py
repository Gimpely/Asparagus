#!/usr/bin/env python3


'''
	Matic Vodopivec
	21.7.2022

	Help:
	Detect asparagus from pointcloud

'''


import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose2D
import math
import tf2_ros
import tf
import numpy as np
import time

class detect_asparagus:
	def __init__(self, topic, z_par, y_limit):
		'''
		Inputs:
			- topic (string) = name of the topic of the pointcloud in robot coordinates
			- z_par (array3)  = parameters for searchnig points in Z direction [start_z, step_z, min_z]
				- start_z = heighest point to start searching for asparagus (should be shortest asparagus height we still need to harvest)
				- step_z  = step to go down in z direction
				- min_z   = height at which we stop searching
			- y_limit (array2) = limits in y minus and y plus direction [y_minus_limit, y_plus_limit]
		'''

		self.start_z = z_par[0]
		self.step_z  = z_par[1]
		self.min_z   = z_par[2]
		self.search_radius = z_par[3]

		self.y_minus_limit = y_limit[0]
		self.y_plus_limit  = y_limit[1]
		
		rospy.init_node("detect_asparagus", anonymous=True)
		# Subscribe to point cloud topic
		rospy.Subscriber(topic, PointCloud, self.callback_pointcloud, queue_size=1)
		# Get current pose of the track robot (calculation is not real time so we need location of the platform when data was acquired)
		rospy.Subscriber("/tracks/pose", Pose2D, self.callback_track_pose)
		self.pub = rospy.Publisher("/aspragus_cloud", PointCloud, queue_size=1)

		self.listener = tf.TransformListener()

	def callback_track_pose(self, data):
		self.track_pose = data

	def callback_pointcloud(self, data):
		self.pointcloud = data.points

		asparagus_points = self.find_z_points()

		robot_points = PointCloud()
		robot_points.header.stamp = rospy.Time.now()
		robot_points.header.frame_id = 'global'
		robot_points.points = asparagus_points

		#transformed_cloud = self.listener.transformPointCloud('global',robot_points)

		self.pub.publish(robot_points)

	def find_z_points(self):
		# Start search for points at start z poz
		z_search = self.min_z #self.start_z

		first_point = True

		# Max number of center points
		max_num_of_point_groups = 50
		max_num_of_points = 100 #(self.start_z - self.min_z) / self.step_z
		num_of_steps = (self.start_z - self.min_z) / self.step_z
		num_of_steps = int(num_of_steps)
		#print(num_of_steps)
		max_num_of_points = int(max_num_of_points)
		center_point = np.zeros((max_num_of_point_groups,2))
		pointArr = np.zeros((max_num_of_point_groups, max_num_of_points, 3))
		current_point =np.zeros(max_num_of_point_groups)

		points_in_range = np.zeros(max_num_of_point_groups)

		# calculate number of points of asparagus based on step_z size
		numOfSteps = (self.start_z - self.min_z) / self.step_z
		asparagus_points = []#np.zeros((10, int(numOfSteps)))
		#print(numOfSteps)

		current_z_step = 0
		numOfAsparagus = 0
		test = 0

		point_in  = 0
		point_out = 0

		# Search for points inside a difined circle. Start search at defined start Z height and go down in steps to min Z height
		while z_search > self.min_z:
			#print("z_search = ", z_search)

			# Set limits in Z direction where we are searching for points inside of current step
			z_up_limit = z_search + (self.step_z) / 2
			z_down_limit = z_search - (self.step_z) / 2

			idx = 0
			center_point_set = np.zeros(max_num_of_point_groups)

			

			#print(self.pointcloud)

			# Go through all points
			for i in self.pointcloud:
				point_in_circle = False
				# Check only for points there limited by Y cooridinate (to eliminate tracks)
				if i.y > self.y_minus_limit and i.y < self.y_plus_limit:
					# Search for point at specified Z range
					if i.z > z_down_limit and i.z < z_up_limit:
						#asparagus_points = np.append(asparagus_points, i)
						test = test + 1
						#print(test)

						
						# Find all asparagus at start z height
						if first_point:
							numOfAsparagus = 1

						# Search inside all defined circles
						for j in range(numOfAsparagus):
							#print(j)
							#print(numOfAsparagus)
							
							max_points_in_range = int(max_num_of_points / num_of_steps)
							#print("Center = ", center_point[j])
							# Search in circle
							if (i.x - center_point[j][0])**2 + (i.y - center_point[j][1])**2 < self.search_radius**2 or first_point:
								
								
								#print("Point in circle = ", i)

								point_in = point_in + 1


								point_in_circle = True

								if points_in_range[j] < max_points_in_range:
									# Check how many points are in defined range
									points_in_range[j] = points_in_range[j] + 1
									# Save data to array
									index = int(current_point[j])
									#print(index)
									pointArr[j][index][0] = i.x
									pointArr[j][index][1] = i.y
									pointArr[j][index][2] = i.z
									current_point[j] = current_point[j] + 1
								
								if center_point_set[j] == 0:
									center_point_set[j] = 1
									asparagus_points = np.append(asparagus_points, i)
								#print(len(asparagus_points))

								if first_point:
									first_point = False
									# Set search range for next point
									
									center_point[0][0] = i.x
									center_point[0][1] = i.y
								
								continue
									
						
						if point_in_circle == False:
							point_out = point_out + 1
							# Define a new circle
							#center_point[numOfAsparagus][0] = i.x
							#center_point[numOfAsparagus][1] = i.y

							#print(center_point[:10])
							
							numOfAsparagus = numOfAsparagus + 1
								
						
						'''
						if point_in_circle == False and first_point == False:
							# Save new center point
							if numOfAsparagus < max_num_of_point_groups:
								center_point[numOfAsparagus][0] = i.x
								center_point[numOfAsparagus][1] = i.y

								#print(center_point[:10])
								
								numOfAsparagus = numOfAsparagus + 1
						'''
							

						
						
			
			
			z_search = z_search + self.step_z
			center_point_set = np.zeros(max_num_of_point_groups)
			points_in_range = np.zeros(max_num_of_point_groups)
			current_z_step = current_z_step + 1
			#break

		print("Num point in  = ", point_in)	
		print("Num point out  = ", point_out)	
		print("Num of asparagus  = ", numOfAsparagus)	
		print("Center = ", center_point)
		
		#print(asparagus_points)

		'''
		#print("Num of detected = ", numOfAsparagus)
		#print(pointArr[0])
		#print(current_point)
		#final_points = []
		multi_point_idx = []
		numOfPointsTotal = 0
		
		for idx,i in enumerate(pointArr):
			if current_point[idx] > 15:
				#print(current_point[idx])
				multi_point_idx = np.append(multi_point_idx, idx)

		if len(multi_point_idx) > 0:
			most_points_idx = np.argmax(multi_point_idx)
			print(pointArr[most_points_idx])
			final_points = pointArr[int(multi_point_idx[most_points_idx])][0]

			
			for i in range(len(multi_point_idx)-1):
				idx = multi_point_idx[i+1]
				for j in pointArr[int(idx)]:
					if j[0] != 0:
						numOfPointsTotal = numOfPointsTotal +1
						final_points = np.vstack((final_points, j))
			

			#idx = multi_point_idx[i+1]
			for j in pointArr[most_points_idx]:
				if j[0] != 0:
					numOfPointsTotal = numOfPointsTotal +1
					final_points = np.vstack((final_points, j))

			#print(final_points)

		#print(len(final_points))
		
		asparagus_points = asparagus_points[:(numOfPointsTotal+1)]
		try:
			for idx, i in enumerate(final_points):
				asparagus_points[idx].x = i[0]
				asparagus_points[idx].y = i[1]
				asparagus_points[idx].z = i[2]
		except:
			pass
		#print(asparagus_points)

		'''
		return asparagus_points



if __name__ == "__main__":
	# Call class
	#topic = "/intel_l515_global_cloud"
	topic = "/scanner_3D_img"
	start_z = 0.25
	step_z = 0.01
	min_z = 0.02
	search_radius = 0.05

	z_par = [start_z, step_z, min_z, search_radius]
	
	y_plus_limit = 0.3
	y_minus_limit = -0.3
	y_limit = [y_minus_limit, y_plus_limit]

	detector = detect_asparagus(topic=topic, z_par=z_par, y_limit=y_limit)
	rospy.spin()