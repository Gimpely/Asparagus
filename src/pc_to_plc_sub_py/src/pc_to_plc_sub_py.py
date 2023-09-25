#!/usr/bin/env python3

import rospy
from beckhoff_msgs.msg import array5

def callback(data):
    rospy.loginfo("I heard %f", data.data[0])


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("alpha_delta_pub", array5, callback)
    rospy.spin()

if __name__=='__main__':
    listener()
