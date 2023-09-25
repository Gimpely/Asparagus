#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CameraInfo

def camera_info_callback(msg):
    # The intrinsic camera matrix for the raw (distorted) images.
    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]
    # Projects 3D points in the camera coordinate frame to 2D pixel
    # coordinates using the focal lengths (fx, fy) and principal point
    # (cx, cy).
    K = msg.K
    fx = K[0]
    fy = K[4]
    cx = K[2]
    cy = K[5]
    print("fx: {}, fy: {}, cx: {}, cy: {}".format(fx, fy, cx, cy))

rospy.init_node('camera_info_node')
rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)
rospy.spin()
