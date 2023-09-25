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
import tf
import numpy as np
import time


#init the node
rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()
#pc2_msg = PointCloud2()

#define the publisher
pc_pub = rospy.Publisher("/robot_cloud", PointCloud, queue_size=1)

listener2 = tf.TransformListener()

scan = LaserScan()


def scan_cb(msg):

    y_limit_min = y_limit[0]
    y_limit_max = y_limit[1]

    scan = msg

    #print('A')

    start_angle = scan.angle_min
    step_angle  = scan.angle_increment
    last_angle  = scan.angle_max
    #print(last_angle)

    filter_angle = math.radians(filter_angle)
    end_angle = start_angle + filter_angle

    current_angle = start_angle

    numOfPoints = 0
    index_array = []

    start_idx = 0
    #print("")
    while current_angle <= last_angle:
        #numOfPoints = 0
        while current_angle <= end_angle:
            current_angle = current_angle + step_angle
            numOfPoints = numOfPoints + 1
            #print(numOfPoints)

        end_idx = numOfPoints
        ranges = scan.ranges[start_idx:end_idx]
        min_idx = np.argmin(ranges)
        min_idx = min_idx + start_idx
        
        #print(min_idx)

        start_idx = numOfPoints

        end_angle = current_angle + filter_angle

        if scan.ranges[min_idx] > 0.1:
            index_array = np.append(index_array, min_idx)
        
    index_array = np.asarray(index_array, dtype=np.uint16)
    #print(index_array)

    # Transform polar coordinates to cartesian
    cartesian_coordinates = np.zeros((len(index_array), 3))
    #cartesian_coordinates = []

    for idx,i in enumerate(index_array):
        trans_angle = start_angle + i * step_angle

        x = scan.ranges[i] * math.cos(trans_angle)
        y = scan.ranges[i] * math.sin(trans_angle)
        z = 0

        cartesian_coordinates[idx][0] = x
        cartesian_coordinates[idx][1] = y
        cartesian_coordinates[idx][2] = z

    cartesian_cloud = PointCloud()
    cartesian_cloud.header.stamp = rospy.Time.now()
    cartesian_cloud.header.frame_id = 'laser'
    cartesian_cloud.points = cartesian_coordinates

    transformed_cloud = listener2.transformPointCloud('robot',cartesian_cloud)
    #print(transformed_cloud.points)

    
    cleaned_points = []
    # Delete points where x is larger or smaller than our boundries
    for idx,i in enumerate(transformed_cloud.points):
        
        if i.y < y_limit_min:
            #print(i.x)
            cleaned_points = np.append(cleaned_points, idx)
        
        if i.y > y_limit_max:
            #print(i.x)
            cleaned_points = np.append(cleaned_points, idx)

    cleaned_points = cleaned_points.astype(int)
    #print(cleaned_points)
    
    transformed_cloud.points = np.delete(transformed_cloud.points, cleaned_points, axis=0)
    

    pc_pub.publish(transformed_cloud)

def cloud(msg):
    start_time = time.time()
    
    pc2_msg = msg

    y_limit_min = y_limit[0]
    y_limit_max = y_limit[1]
    z_min_limit = 0.02#.06
    z_max_limit = 0.4

    # or a list of the individual points which is less efficient
    point_list = pc2.read_points_list(pc2_msg)
    
    local_points = PointCloud()
    local_points.header.stamp = rospy.Time.now()
    local_points.header.frame_id = 'laser'
    local_points.points = point_list

    transformed_cloud = listener2.transformPointCloud('robot',local_points)
    #pc_pub.publish(transformed_cloud)
    #print(pc22)


    

    cleaned_points = []
    # Delete points where x is larger or smaller than our boundries
    for idx,i in enumerate(transformed_cloud.points):
        
        if i.y < y_limit_min or i.y > y_limit_max or i.z < z_min_limit or i.z > z_max_limit:
            cleaned_points = np.append(cleaned_points, idx)
        
        #if i.y > y_limit_max:
        #    cleaned_points = np.append(cleaned_points, idx)

    cleaned_points = cleaned_points.astype(int)
    
    transformed_cloud.points = np.delete(transformed_cloud.points, cleaned_points, axis=0)

    #print(transformed_cloud)

    pc_pub.publish(transformed_cloud)
    
    #print(len(transformed_cloud.points))
    #print("Elapsed time = ", time.time() - start_time)

    
if __name__ == "__main__":

    topic = "/cloud"
    y_limit = [-0.2, 0.2]
    filter_angle = 5

    

    if topic == "/cloud":
        rospy.Subscriber("/cloud", PointCloud2, cloud, queue_size=1)
    else:
        rospy.Subscriber("/scan", LaserScan, scan_cb, queue_size=1)

    
    rospy.spin()
