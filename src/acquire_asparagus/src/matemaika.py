#!/usr/bin/env python3
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np

def publish_asparagus_locations():
    rospy.init_node('asparagus_locations_publisher', anonymous=True)

    asparagus_locations_pub = rospy.Publisher('/asparagus_locations', numpy_msg(Floats), queue_size=10)

    while not rospy.is_shutdown():
        # Generate four random values for the asparagus location
        pick_points =  [[-0.01073537,  0.10544874 , 0.04 ,       1.        ],
                        [ 0.15214694, -0.18182003 , 0.04  ,      1.        ],
                        [ 0.3983155,   0.03418354 , 0.04  ,      1.        ],
                        [ 0.5058359,  -0.10434435 , 0.04   ,     1.        ],
                        [ 0.7186424,  -0.00210931 , 0.04    ,    1.        ],
                        [ 0.9802874,  -0.08171631 , 0.04    ,    1.        ]]

        asparagus_point = np.asarray(pick_points, dtype=np.float32)  # Replace with your own values
        asparagus_locations_pub.publish(asparagus_point.flatten())

        


if __name__ == '__main__':
    try:
        publish_asparagus_locations()
    except rospy.ROSInterruptException:
        pass
