#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
import time

def compute_odometry():
    rospy.init_node('dummy_odometry_publisher', anonymous=True)

    pose_pub = rospy.Publisher('/tracks/pose', Pose2D, queue_size=10)

    pose_msg = Pose2D()
    pose_msg.x = 0.0  # Initial x position
    pose_msg.y = 0.0  # Initial y position
    pose_msg.theta = 0.0  # Initial heading angle

    prev_time = time.time()  # Previous loop time

    while not rospy.is_shutdown():

        current_time = time.time()  # Current loop time
        dt = current_time - prev_time  # Time difference between loops
        

        pose_msg.x += 0.1 * dt  # Constant speed of 0.1 m/s in the x direction
        pose_pub.publish(pose_msg)

        prev_time = current_time

        
if __name__ == '__main__':
    try:
        compute_odometry()
    except rospy.ROSInterruptException:
        pass
