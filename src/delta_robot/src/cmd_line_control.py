#!/usr/bin/env python3
import rospy
from rospy_tutorials.msg import Floats
import numpy as np
from rospy.numpy_msg import numpy_msg


def main():
    rospy.init_node('point_publisher')
    # pub = rospy.Publisher('/r_control', Floats, queue_size=1)
    pub = rospy.Publisher('/aspragus_locations', numpy_msg(Floats), queue_size=1)

    # Define hardcoded coordinates for 5 points
    # points = [(0.0, 0.0, -800.0), (150.0, 0.0, -800.0), (150.0, 100.0, -800.0), 
    # (150.0, -100.0, -800.0), (250.0, 0.0, -800.0),(250.0, 100.0, -800.0),(250.0, -100.0, -800.0),
    # (300.0, -0.0, -800.0),(300.0, 50.0, -800.0),(300.0, -50.0, -800.0),(350.0, 0.0, -800.0)]

    # points = [(0.0, 0.0, -760.0), (190.0, 0.0, -780.0), (190.0, 70.0, -780.0), (190.0, 140.0, -780.0),(190.0, -70.0, -780.0), (190.0, -130.0, -780.0),
    # (260.0, 0.0, -780.0),(260.0, 70.0, -780.0),(260.0, 140, -780.0),(260.0, -70.0, -780.0),(260.0, -130.0, -780.0),
    # (285.0, 0.0, -780.0),(285.0, 70.0, -780.0),(285.0, 140.0, -780.0),(285.0, -70.0, -780.0),(285.0, -130.0, -780.0),
    # (306.0, 0.0, -730.0),(306.0, 70.0, -730.0),(306.0, 100.0, -730.0),(306.0, -70.0, -730.0),(306.0, -130.0, -730.0)]

    # points = [(0.0, 0.0, -760.0), (167.0, 0.0, -817.0), (167.0, 70.0, -817.0), (167.0, 140.0, -817.0),(0.0, 0.0, -760.0),(167.0, -70.0, -817.0), (167.0, -140.0, -817.0),
    # (200.0, 0.0, -780.0), (200.0, 70.0, -780.0), (200.0, 140.0, -780.0),(200.0, -70.0, -780.0), (200.0, -140.0, -780.0),
    # (260.0, 0.0, -780.0),(260.0, 70.0, -780.0),(260.0, 140, -780.0),(260.0, -70.0, -780.0),(260.0, -140.0, -780.0),
    # (285.0, 0.0, -780.0),(285.0, 70.0, -780.0),(285.0, 140.0, -780.0),(285.0, -70.0, -780.0),(285.0, -140.0, -780.0),
    # (306.0, 0.0, -730.0),(306.0, 70.0, -730.0),(306.0, 100.0, -730.0),(306.0, -70.0, -730.0),(306.0, -130.0, -730.0)]

    points = np.asarray([ 0.51,  -0.01291,  0.04,        True        ])

    point_index = 0

    while not rospy.is_shutdown():
        try:
            input("Press Enter to publish the next 3D point coordinates...")

            # x, y, z = points[point_index % len(points)]  # Loop over the points
            
            points = np.asarray(points, dtype=np.float32)
            pub.publish(points)

            # point_index += 1  # Move to next point
        except Exception as e:
            print("An error occurred: ", str(e))

if __name__ == '__main__':
    main()



# def main():
#     rospy.init_node('point_publisher')
#     pub = rospy.Publisher('/r_control', Floats, queue_size=1)

#     while not rospy.is_shutdown():
#         try:
#             x, y, z = map(float, input("Enter the 3D point coordinates (x, y, z) separated by spaces: ").split())
#             if z > -700:
#                 print("Invalid z, please try again.")
#                 continue
#             # if x > 200:
#             #     print("Invalid x, please try again.")
#             #     continue
            
#             point = Floats()
#             point.data = [x, y, z, 0, 0]
#             pub.publish(point)
#         except Exception as e:
#             print("Invalid input, please try again.")
#             continue

# if __name__ == '__main__':
#     main()
