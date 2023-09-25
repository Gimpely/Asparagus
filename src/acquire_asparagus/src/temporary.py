#!/usr/bin/env python3
import numpy as np


cleaned1 = np.array([[0.47497174, -0.06051115, 0.02, 1.]])

picked1 = np.array([[0,0,0,0]])

cleaned2 = np.array([[0.4826467, -0.06027853, 0.02, 1.],
 [0.5036532,0.10121465,0.02,1.]])

picked2 = np.array([[0.47857991, -0.06090485 , 0.02,0. ]])

cleaned3 = np.array([[0.50115156, -0.06560285,0.02,1.],
 [ 0.496231, 0.10548463 , 0.02, 1.],
 [ 0.6452826, 0.02044321,  0.02, 1.]])

picked3 = np.array([[0.47857991, -0.06090485 , 0.02, 0.],
 [ 0.50463778,  0.10157 , 0.02 ,0.]])


cleaned4 = np.array([[0.6331236, 0.01917029,  0.02, 1.],
 [0.91192937 ,-0.10043388 , 0.02, 1.]])

picked4 = np.array([[ 0.47857991, -0.06090485,  0.02, 0. ],
 [ 0.50463778 , 0.10157 ,    0.02, 0.  ],
 [ 0.64458454 , 0.02044313,  0.02, 0. ]])

cleaned5 = np.array([[0.91192454, -0.09954032, 0.02, 1.]])

picked5 = np.array([[ 0.47857991 ,-0.06090485 , 0.02, 0.],
 [ 0.50463778,  0.10157 ,    0.02, 0. ],
 [ 0.64458454, 0.02044313 , 0.02, 0.  ],
 [ 0.91306895 ,-0.10036136,  0.02 , 0. ]])


def main(cleaned, picked):
    detection_array = cleaned

    picked_array = picked

    flag_array = np.zeros(len(detection_array), dtype=bool)
    distance_threshold = 0.025

    for i, det_point in enumerate(detection_array):
        flag_array[i] = False  
        for pick_point in picked_array:
            distance = np.linalg.norm(det_point[:3] - pick_point[:3])
            if distance <= distance_threshold:
                flag_array[i] = True
                break

    # Print the flag array
    print("flag",flag_array)

if __name__ == "__main__":
    # main(cleaned1, picked1)
    # main(cleaned2, picked2)
    main(cleaned3, picked3)
    # main(cleaned4, picked4)
    # main(cleaned5, picked5)

