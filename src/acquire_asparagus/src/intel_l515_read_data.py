#!/usr/bin/env python3


'''
	Matic Vodopivec
	14.7.2022

	Help:
	- Reading data from Intel realsense L515
    - Transforming data to "robot" coordinate frame, setting limits in Y direction
    - Transforming data to "global" coordinate frame
    - Saving 3D point cloud to file

'''

from distutils.command.clean import clean
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan, PointCloud
import laser_geometry.laser_geometry as lg
import math
import tf2_ros
import tf
import numpy as np
import time

rospy.init_node("intel_l515_to_pointcloud", anonymous=True)

pc_pub = rospy.Publisher("/intel_l515_global_cloud", PointCloud, queue_size=1)

listener2 = tf.TransformListener()

def transform(msg):
    start_time = time.time()

    cloud_points = list(pc2.read_points(msg, skip_nans=True, field_names = ("x", "y", "z")))
    #print(cloud_points[:][0])

    robot_points = PointCloud()
    robot_points.header = msg.header
    robot_points.points = cloud_points
    #print(robot_points.points[0])

    
    transformed_cloud = listener2.transformPointCloud('robot',robot_points)

    '''
    cleaned_points = []
    # Delete points where x is larger or smaller than our boundries
    for idx, i in enumerate(transformed_cloud.points):
        
        if i.z < 0.1: #or i.y > 0.2:
            #print(i.x)
            cleaned_points = np.append(cleaned_points, idx)
        
        #if i.y > 0.1: #or i.y > 0.2:
            #print(i.x)
            #cleaned_points = np.append(cleaned_points, idx)


    cleaned_points = np.asarray(cleaned_points, dtype=np.uint16)
    
    # Delete points
    transformed_cloud.points = np.delete(transformed_cloud.points, cleaned_points, axis=0)
    print(transformed_cloud.points[0])
    '''

    #print("First points ")
    #print(transformed_cloud.points[:10])
    #print("Last points ")
    #print(transformed_cloud.points[-10:])
    
    pc_pub.publish(transformed_cloud)

    end = time.time()
    print(end - start_time)
    

        
rospy.Subscriber("/camera/depth/color/points", PointCloud2, transform, queue_size=1)
rospy.spin()
