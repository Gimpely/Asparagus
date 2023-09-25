#!/usr/bin/env python3

'''
	Matic Vodopivec
	12.7.2022

	Help:
	- Read data from sick scanner transformed into world CS
	- Read pose from track robot
	- Assemble 3D image
'''

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import PointCloud
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
import tf
from tf2_ros import TransformBroadcaster
import time
from geometry_msgs.msg import TwistStamped
 
class get_3D_sensor_img: 
	def __init__(self):
		rospy.init_node('sensor_img')
		
		# Get pose 2D of the tracks
		rospy.Subscriber("/tracks/pose", Pose2D, self.read_track_pose)
		self.track_pose = Pose2D()
		self.save_poz = False
		# Get 3D point cloud from sick sensor in world CS
		rospy.Subscriber("/robot_cloud", PointCloud, self.get_world_point_cloud)
		self.current_point_cloud = PointCloud()
		self.scanner_cloud_points = []
		self.world_point_cloud = PointCloud()
		self.save_start_poz = False
		self.new_point_cloud = PointCloud()

		self.save_increment = 0.001

		# Read actual linear and angular velocity of tracks
		rospy.Subscriber("/tracks/twist_state", TwistStamped, self.callback_twist)

		self.listener = tf.TransformListener()

		self.pc_pub = rospy.Publisher("/scanner_3D_img", PointCloud, queue_size=1)

		# Initialize the transform broadcaster
		self.br = tf.TransformBroadcaster()

		# Define max array length
		self.max_array_len = 100
		self.num_of_points_arr = np.zeros(self.max_array_len)
		self.arr_idx = 0

		self.route = 0
		self.counter = 0
		self.dT = 0.001


	def callback_twist(self,data):
		self.twist_data = data
		self.track_linear_vel = self.twist_data.twist.linear.x

	def read_track_pose(self, data):
		self.track_pose = data

	def get_world_point_cloud(self, data):
		#start_time = time.time()

		# Read points in track coordinate system
		self.current_point_cloud = PointCloud()
		self.current_point_cloud = data

		# transform data to global CS
		self.world_point_cloud = self.listener.transformPointCloud('global',self.current_point_cloud)

		if self.save_poz == False:
			
			self.saved_track_pose = self.track_pose

			self.save_poz = True

		# Measure cycle time
		if self.counter == 0:
			self.start_time = time.time()
			self.counter = 1
		else:
			self.dT = time.time() - self.start_time
			self.counter = 0

		if self.track_linear_vel > 0:
			self.route = self.route + self.dT * self.track_linear_vel
		#print(self.route)

		route_x = self.track_pose.x - self.saved_track_pose.x
		#route_y = self.track_pose.y - self.saved_track_pose.y

		self.new_point_cloud.header.stamp = rospy.Time.now()
		self.new_point_cloud.header.frame_id = 'global'



		# Save data to array each time track robot move for more than 1 mm
		if self.route > self.save_increment:
			print("Platform linear velocity = ", str(self.track_linear_vel*100) + " cm/s")
			self.route = 0

			self.num_of_points_arr[self.arr_idx] = len(self.world_point_cloud.points)

			self.arr_idx = self.arr_idx + 1

			# Save points from scanner to array			
			self.scanner_cloud_points = np.append(self.scanner_cloud_points, self.world_point_cloud.points)			

			if self.arr_idx > self.max_array_len-1:
				# Delete n points from arrray
				num_of_points_first_arr = int(self.num_of_points_arr[0])
				#print("num_of_points_first_arr = ", num_of_points_first_arr)
				#print(num_of_points_first_arr)
				self.scanner_cloud_points = self.scanner_cloud_points[num_of_points_first_arr:]

				#print(len(self.scanner_cloud_points))

				self.num_of_points_arr[0] = 0
				self.num_of_points_arr = np.roll(self.num_of_points_arr, -1)

				self.arr_idx = self.max_array_len - 1

			# Save points in reversed order
			scanner_cloud_points_reversed = self.scanner_cloud_points[::-1]
			
			self.new_point_cloud.points = scanner_cloud_points_reversed

			#print(self.new_point_cloud.points)

			self.save_poz = False

		# Publish data constantly
		self.pc_pub.publish(self.new_point_cloud)
		
		#print("Elapsed time = ", time.time() - start_time)


if __name__ == "__main__":
	# Call class
	sensor_img = get_3D_sensor_img()

	rospy.spin()
