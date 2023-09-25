#!/usr/bin/env python3
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan, PointCloud
import laser_geometry.laser_geometry as lg
import math
import tf
import numpy as np
import time
import std_msgs.msg


class PointCloudProcessor:
    def __init__(self):
        # Set up the ROS subscriber and publisher
        self.subscriber = rospy.Subscriber('/cloud', PointCloud2, self.process_pointcloud)
        self.publisher = rospy.Publisher('/robot_cloud', PointCloud2, queue_size=10)


    def process_pointcloud(self, pointcloud):
        y_limit_min = y_limit[0]
        y_limit_max = y_limit[1]
        z_min_limit = 0.06
        z_max_limit = 0.4
        # Convert PointCloud2 message to a list of points
        local_points = list(pc2.read_points(pointcloud, field_names=("x", "y", "z"), skip_nans=True))

        # cleaned_points = []
        # # Delete points where x is larger or smaller than our boundaries
        # for idx, i in enumerate(local_points):
        #     if i[1] < y_limit_min or i[1] > y_limit_max or i[2] < z_min_limit or i[2] > z_max_limit:
        #         cleaned_points = np.append(cleaned_points, idx)


        # cleaned_points = cleaned_points.astype(int)
        
        # local_points = np.delete(local_points, cleaned_points, axis=0)
        # Convert the filtered points back to PointCloud2 message
        filtered_pointcloud = pc2.create_cloud_xyz32(pointcloud.header, local_points)
        self.publisher.publish(filtered_pointcloud)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pointcloud_processor')
    topic = "/cloud"
    y_limit = [-0.2, 0.2]
    filter_angle = 5
    processor = PointCloudProcessor()
    rospy.spin()



