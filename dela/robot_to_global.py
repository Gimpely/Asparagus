#!/usr/bin/env python3
import rospy

# Because of transformations
#import tf_conversions

import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler


def robot_to_global(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # print("Msg1: ", msg)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "global"
    t.child_frame_id = 'robot'
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)


    br1 = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # print("Msg2: ", msg)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "robot"
    t.child_frame_id = 'global'
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # br1.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    rospy.Subscriber("/tracks/pose", geometry_msgs.msg.Pose2D, robot_to_global)
    rospy.spin()