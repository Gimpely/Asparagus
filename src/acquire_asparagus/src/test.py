#!/usr/bin/env python3
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointCloud
from sensor_msgs import point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, PoseStamped
import rospy
import copy


class PointCloudVisualizer:
    def __init__(self):
        rospy.init_node('point_cloud_visualizer')
        # Set up the ROS subscriber for the point cloud data
        # self.pc_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback, queue_size=1)
        self.pc_sub = rospy.Subscriber('/cloud', PointCloud2, self.callback, queue_size=1)
        # self.pc_sub = rospy.Subscriber('/robot_cloud"', PointCloud2, self.callback, queue_size=1)
        self.points = []


    def callback(self, pc_msg):
        

        # Extract the points from the point cloud message
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(pc_msg)])

        # Create an Open3D point cloud object from the points
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        
        print("")
        print(
            "1) Please pick at least three correspondences using [shift + left click]"
        )
        print("   Press [shift + right click] to undo point picking")
        print("2) After picking points, press 'Q' to close the window")
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcd)
        vis.run()  # user picks points
        vis.destroy_window()
        print("")
        print(vis.get_picked_points())
        self.points.append(pcd.points[vis.get_picked_points()[0]])
        pointsArray = np.array(self.points)
        avg_coordinates = np.mean(pointsArray, axis=0)
        print(pcd.points[vis.get_picked_points()[0]])
        print("Average Coordinates (x, y, z):", avg_coordinates)
        

        

if __name__ == '__main__':
    # Initialize the ROS node
    

    # Create an instance of the PointCloudVisualizer class
    visualizer = PointCloudVisualizer()

    # Spin the ROS node to process incoming messages
    rospy.spin()

