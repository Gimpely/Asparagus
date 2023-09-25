#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped, Point
from laser_geometry import LaserProjection
from std_msgs.msg import Header
from pynput import keyboard   
import time 
from rospy_tutorials.msg import Floats
 
# Global variables
lidar_data = None
robot_positions = []
height = None
r_control = None
key_pressed = None

# Detect the stick point in the lidar frame
def detect_stick_point(lidar_ranges, angle_increment, start_index):
    global height
    min_range_index = np.argmin(lidar_ranges)
    min_range = lidar_ranges[min_range_index]
    min_range_angle = (start_index + len(lidar_ranges) -1 - min_range_index) * angle_increment + 50*np.pi/180

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
    lidar_data = np.array(msg.ranges)
    return

def robot_position_callback( data):
    global r_control
    r_control = data.data
    r_control = r_control[:3]
    return


def on_press(key):
    global key_pressed
    try:
        if key.char in ('a', 'b'):
            key_pressed = key.char
        else:
            key_pressed = None
    except AttributeError:
        pass



def kabsch(P, Q):
    """
    Calculate the optimal rotation matrix using the Kabsch algorithm.

    Args:
        P (numpy.array): A Nx3 matrix of reference 3D points.
        Q (numpy.array): A Nx3 matrix of target 3D points.

    Returns:
        (numpy.array): A 3x3 optimal rotation matrix.
    """
    assert P.shape == Q.shape

    # Calculate the covariance matrix
    C = np.dot(np.transpose(P), Q)

    # Compute the Singular Value Decomposition (SVD)
    U, _, Vt = np.linalg.svd(C)

    # Compute the rotation matrix
    d = (np.linalg.det(U) * np.linalg.det(Vt)) < 0.0
    R = np.dot(U, np.dot(np.diag([1, 1, -1 if d else 1]), Vt))

    return R

def get_transform(Pl, Pr):
    """
    Calculate the transformation matrix between the coordinate systems
    of two sets of 3D points.

    Args:
        Pl (list): A list of reference 3D points.
        Pr (list): A list of target 3D points.

    Returns:
        (numpy.array): A 4x4 transformation matrix.
    """
    assert len(Pl) == len(Pr)

    # Convert the input lists to NumPy arrays
    Pl = np.array(Pl)
    Pr = np.array(Pr)

    # Calculate the centroids of both point sets
    Pl_centroid = np.mean(Pl, axis=0)
    Pr_centroid = np.mean(Pr, axis=0)

    # Center the point sets by subtracting their centroids
    Pl_centered = Pl - Pl_centroid
    Pr_centered = Pr - Pr_centroid

    # Calculate the optimal rotation matrix using the Kabsch algorithm
    R = kabsch(Pl_centered, Pr_centered)

    # Calculate the translation vector
    t = Pr_centroid - np.dot(R, Pl_centroid)

    # Assemble the transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T


# Main function
def main():
    rospy.init_node("hand_eye_calibration")
    global height
    global r_control
    global key_pressed
    recorded_points_lasr = []
    recorded_points = []

    angle_increment = 0.004363323096185923
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    # rospy.Subscriber("/r_control", PointStamped, robot_position_callback)
    rospy.Subscriber("/r_control", Floats, robot_position_callback, queue_size=1)


    
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    while not rospy.is_shutdown():
        if lidar_data is not None:
            # Crop the scan to include only the middle part
            start_index = int(0.27 * len(lidar_data))
            end_index = int(0.80 * len(lidar_data))
            cropped_lidar_data = lidar_data[start_index:end_index]
            
            stick_point = detect_stick_point(cropped_lidar_data, angle_increment, start_index)

            # if key_pressed == 'a':
            #     if len(recorded_points_lasr) < 10 and len(recorded_points) < 10:
            #         recorded_points.append(r_control)
            #         recorded_points_lasr.append(stick_point)
            #         print("Recorded point", len(recorded_points))
            #     else:
            #         print("Maximum number of points reached.")
            #     key_pressed = None
            # elif key_pressed == 'b':
            #     print("calculate---------------------------")
            #     T = get_transform(recorded_points_lasr, recorded_points)
            #     print("T: ", T)

            #     listener.stop()
            #     break


            # print("Press 'a' to record a point, or 'b' to perform calculation and exit.")
            # time.sleep(0.1)

        

if __name__ == "__main__":
    main()
