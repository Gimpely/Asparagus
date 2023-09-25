#!/usr/bin/env python3


'''
	Matic Vodopivec
	21.7.2022

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
from math import cos, sin, radians

def trig(angle):
    r = radians(angle)
    return cos(r), sin(r)

def matrix(rotation, translation):
    xC, xS = trig(rotation[0])
    yC, yS = trig(rotation[1])
    zC, zS = trig(rotation[2])
    dX = translation[0]
    dY = translation[1]
    dZ = translation[2]
    Translate_matrix = np.array([[1, 0, 0, dX],
                               [0, 1, 0, dY],
                               [0, 0, 1, dZ],
                               [0, 0, 0, 1]])
    Rotate_X_matrix = np.array([[1, 0, 0, 0],
                              [0, xC, -xS, 0],
                              [0, xS, xC, 0],
                              [0, 0, 0, 1]])
    Rotate_Y_matrix = np.array([[yC, 0, yS, 0],
                              [0, 1, 0, 0],
                              [-yS, 0, yC, 0],
                              [0, 0, 0, 1]])
    Rotate_Z_matrix = np.array([[zC, -zS, 0, 0],
                              [zS, zC, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
    return np.dot(Rotate_Z_matrix,np.dot(Rotate_Y_matrix,np.dot(Rotate_X_matrix,Translate_matrix)))

tf_matrix = matrix([0, 135, 0], [0.7, 0.1, 0.4])
print(tf_matrix)

rospy.init_node("intel_l515_to_pointcloud", anonymous=True)

pc_pub = rospy.Publisher("/intel_l515_global_cloud", PointCloud, queue_size=1)

listener2 = tf.TransformListener()

def transform(msg):
    start_time = time.time()

    pc2_msg = msg
    
    # convert it to a generator of the individual points
    cloud_points = list(pc2.read_points(pc2_msg, skip_nans=True, field_names = ("x", "y", "z")))
    cloud_points_array = np.asarray(cloud_points)
    
    cloud_points_robot = np.zeros_like(cloud_points_array)
    for idx,i in enumerate(cloud_points_array):
        # Add 1 to i
        i = np.append(i, 1)
        #print("i", i)
        transformed_point = np.dot(tf_matrix, np.transpose(i))
        transformed_point = transformed_point[:3]
        cloud_points_robot[idx] = transformed_point
        #print(transformed_point)
    
    #print(cloud_points_robot)

    #print(len(cloud_points_array))

    
    # Transform PointCloud2 to PointCloud
    robot_points = PointCloud()
    robot_points.header = pc2_msg.header
    robot_points.points = cloud_points_robot
    
    # Transfrom point cloud from camera frame to robot frame
    #transformed_cloud = listener2.transformPointCloud('robot',robot_points)
    '''
    # Check which points are out of range
    cleaned_points = []
    # Delete points where x is larger or smaller than our boundries
    for idx, i in enumerate(transformed_cloud.points):
        if i.y < -0.20 or i.y > 0.2:
            #print(i.x)
            cleaned_points = np.append(cleaned_points, idx)
        
        #if i.y > 0.26: #or i.y > 0.2:
            #print(i.x)
            #cleaned_points = np.append(cleaned_points, idx)

        #print(transformed_cloud.points[idx].y)
        #cleaned_point = np.where(transformed_cloud.points[idx].y < -0.2 or transformed_cloud.points[idx].y > 0.26 for idx in transformed_cloud.points)
        
        #cleaned_points = np.append(cleaned_points, cleaned_points)

    #print("Cleaned points =  ", cleaned_points)


    #cleaned_points = cleaned_points.astype(int)
    #print("Cleaned points =  ", cleaned_points)

    cleaned_points = np.asarray(cleaned_points, dtype=np.uint16)
    print(cleaned_points)
    
    
    #transformed_cloud.points = np.delete(transformed_cloud.points, cleaned_points, axis=0)

    '''
    
    pc_pub.publish(robot_points)

    end = time.time()
    print(end - start_time)
    


        
rospy.Subscriber("/camera/depth/color/points", PointCloud2, transform, queue_size=1)
rospy.spin()
