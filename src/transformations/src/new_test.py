#!/usr/bin/env python3

import rospy
# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    
    rospy.init_node('laser_to_global_tf', anonymous=True)
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "global"
    static_transformStamped.child_frame_id = "laser"

    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0

    quat = tf.transformations.quaternion_from_euler(float(0),float(2.36),float(0))#float(3.14159))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()