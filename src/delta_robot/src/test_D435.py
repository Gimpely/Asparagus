#!/usr/bin/env python3

# First import the library
import pyrealsense2 as rs
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import time


# Define publisher
#print('A')
rospy.init_node("test2")
pc_pub = rospy.Publisher("/intel_cloud", numpy_msg(Floats), queue_size=1)

try:
    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()

    # Configure streams
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Start streaming
    pipeline.start(config)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        start_time = time.time()
        #a = numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32)
        # This call waits until a new coherent set of frames is available on a device
        # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()

        pc = rs.pointcloud()
        points = pc.calculate(depth)
        color = frames.get_color_frame()
        #print(points[0])
        pc.map_to(color)

        print(pc)

        if not depth: continue

        #print(depth)

        # Convert message to np array
        depth_image = np.asanyarray(depth.get_data(), dtype=np.float32)
        pc_pub.publish(depth_image)

        end_time = time.time()

        #print(depth_image)

        print(end_time - start_time)

        
        r.sleep()

except Exception as e:
    print(e)
    pass
