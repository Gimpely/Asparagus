#!/usr/bin/env python3


'''
	Matic Vodopivec
	8.7.2022

	Help:
	Read data from sick scanner LMS111

'''

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan, PointCloud
import laser_geometry.laser_geometry as lg
import math
import tf2_ros
import tf

rospy.init_node("test")

lp = lg.LaserProjection()
#pc2_msg = PointCloud2()

pc_pub = rospy.Publisher("/global_cloud", PointCloud, queue_size=1)

listener2 = tf.TransformListener()

def scan_cb(msg):
	# convert the message of type LaserScan to a PointCloud2
	#pc2_msg = lp.projectLaser(msg) #, channel_options=)
	pc2_msg = msg
	#print(pc2_msg)

	# now we can do something with the PointCloud2 for example:
	# publish it
	#pc_pub.publish(pc2_msg)

	#tfBuffer = tf2_ros.Buffer()
	#listener = tf2_ros.TransformListener(tfBuffer)
	#trans = tfBuffer.lookup_transform('global', 'robot', rospy.Time(0))
	#print(trans)

	#listener2 = tf.TransformListener()

	try:
		pc22 = listener2.transformPointCloud('global',pc2_msg)
		#print(pc22)
		pc_pub.publish(pc22)
	except:
		pass
	

		
rospy.Subscriber("/robot_cloud", PointCloud, scan_cb, queue_size=1)
#rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()
