#!/usr/bin/env python3
import numpy as np



def convert_robot_cs_to_point(point, z_offset):
        

    point_robot = np.zeros(len(point))

    temp= np.divide(point[:3],1000)

    point_robot[:3] = temp
    point_robot[2] = point[2]- z_offset
    point_robot[2] = point_robot[2]/1000
    point_robot[3:] = point[3:]

    return point_robot


def convert_point_to_robot_cs(point, z_offset):
    point_robot = np.zeros(len(point))
    
    temp= np.multiply(point[:3],1000)
    point_robot[:3] = temp #point[:3] * 1000
    point_robot[2] = point[2]*1000 + z_offset
    point_robot[3:] = point[3:]

    return point_robot


z_offset = -900
next_poz = [100, 30, -900]

point1 = convert_robot_cs_to_point(next_poz, z_offset)
print(point1)
point2 = convert_point_to_robot_cs(point1, z_offset)
print(point2)

print(np.divide(next_poz,1))