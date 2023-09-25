#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D

sim_pose2d = Pose2D()

def talker():
    pub = rospy.Publisher('/tracks/pose', Pose2D, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    sim_pose2d.x = 0.3
    sim_pose2d.y = 0.05
    sim_pose2d.theta = 0

    while not rospy.is_shutdown():
        
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish(sim_pose2d)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass