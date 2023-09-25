#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped, Point
from laser_geometry import LaserProjection
from std_msgs.msg import Header


# Global variables
lidar_data = None
robot_positions = []
height = None
r_control = None
lidar_msg = None

p1 = [0.54262168, 0.13604807, 0]


# Detect the stick point in the lidar frame
def detect_stick_point(lidar_ranges, angle_increment, start_index):
    global height
    min_range_index = np.argmin(lidar_ranges)
    min_range = lidar_ranges[min_range_index]
    min_range_angle = (start_index + len(lidar_ranges) -1 - min_range_index) * angle_increment + 50*np.pi/180
    # print(min_range_angle)
    min_range_x = min_range * np.sin(min_range_angle)
    min_range_y = min_range * np.cos(min_range_angle)

    # print("range", min_range)
    h = min_range *1000*np.cos(np.pi/4)
    height = 555-h

    min_range_z = 0#  -0.185  # The tip of the calibration stick is 18.5 cm below the TCP
    # print(min_range_angle * 180/np.pi)
    return np.array([min_range_x, min_range_y, min_range_z])

# Callback functions
def lidar_callback(msg):
    global lidar_data
    global lidar_msg
    lidar_msg = msg
    lidar_data = np.array(msg.ranges)

def robot_position_callback(msg):
    global robot_positions
    global r_control
    r_control = msg.point.z
    
    robot_positions.append(np.array([msg.point.x, msg.point.y, msg.point.z]))

# Main function
def main():
    rospy.init_node("point_recorder")
    global height

    angle_increment = 0.004363323096185923
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    rospy.Subscriber("/r_control", PointStamped, robot_position_callback)

    stick_point_pub = rospy.Publisher("/detected_stick_point", PointStamped, queue_size=1)


    cropped_scan_pub = rospy.Publisher("/cropped_scan", LaserScan, queue_size=1)

    point1_pub = rospy.Publisher("/avg_point1", PointStamped, queue_size=1)
    point4_pub = rospy.Publisher("/avg_point4", PointStamped, queue_size=1)

    rate = rospy.Rate(20)  # 1 Hz
    stick_points = []
    while not rospy.is_shutdown():
        if lidar_data is not None:
            # Crop the scan to include only the middle part
            start_index = int(0.27 * len(lidar_data))
            end_index = int(0.80 * len(lidar_data))

            # start_index = int(0.39 * len(lidar_data))   #0.39 za 2  
            # end_index = int(0.50 * len(lidar_data))     #0.50 za 2  

            # start_index = int(0.27 * len(lidar_data))   #za 1
            # end_index = int(0.40 * len(lidar_data))     #

            # start_index = int(0.50 * len(lidar_data))   #za 3
            # end_index = int(0.70 * len(lidar_data))     #

            # start_index = int(0.68 * len(lidar_data))   #za 4
            # end_index = int(0.80 * len(lidar_data))     #
            cropped_lidar_data = lidar_data[start_index:end_index]

            

            try:
                stick_point = detect_stick_point(cropped_lidar_data, angle_increment, start_index)
                stick_points.append(stick_point)
                average_stick_point = sum(stick_points) / len(stick_points)

                # print("Stick point in lidar frame:", stick_point)
                print("Average stick point in lidar frame:", average_stick_point)
                cropped_scan_msg = LaserScan()
                cropped_scan_msg.header = lidar_msg.header
                cropped_scan_msg.angle_min = lidar_msg.angle_min + start_index * lidar_msg.angle_increment
                cropped_scan_msg.angle_max = lidar_msg.angle_min + end_index * lidar_msg.angle_increment
                cropped_scan_msg.angle_increment = lidar_msg.angle_increment
                cropped_scan_msg.time_increment = lidar_msg.time_increment
                cropped_scan_msg.scan_time = lidar_msg.scan_time
                cropped_scan_msg.range_min = lidar_msg.range_min
                cropped_scan_msg.range_max = lidar_msg.range_max
                cropped_scan_msg.ranges = cropped_lidar_data
                cropped_scan_pub.publish(cropped_scan_msg)


                # Publish the detected stick point for visualization
                point_msg = PointStamped() 
                point_msg.header.stamp = rospy.Time.now()
                point_msg.header.frame_id = "laser"
                point_msg.point = Point(*stick_point)
                stick_point_pub.publish(point_msg)
                

                p1 = [0.544, 0.1484, 0]
                p2 = [0.54249, -0.142665, 0]
                p3 = [0.54249, -0.142665, 0]
                p4 = [0.5409, 0.1395, 0]
                point1_msg = PointStamped() 
                point1_msg.header.stamp = rospy.Time.now()
                point1_msg.header.frame_id = "laser"
                point1_msg.point = Point(*p1)
                point1_pub.publish(point1_msg)
                point4_msg = PointStamped() 
                point4_msg.header.stamp = rospy.Time.now()
                point4_msg.header.frame_id = "laser"
                point4_msg.point = Point(*p4)
                point4_pub.publish(point4_msg)
               
            except Exception as e:
                print("Error:", e)

if __name__ == "__main__":
    main()
