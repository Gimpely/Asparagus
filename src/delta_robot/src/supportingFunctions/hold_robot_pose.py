#!/usr/bin/env python3


'''
	Matic Vodopivec
	5.7.2022

	Help:
	Hold robot still relative to the ground (robot compensating tracks velocity)



'''


import rospy


import numpy as np

def hold_robot_poz(current_poz, linear_velocity, angular_velocity, dt):
    '''
    Inputs:
        - current_poz -> Current robot position (np.array of lenght 5)
        - linear_velocity -> Linear velocity of track robot
        - angular_velocity -> Angular velocity of track robot
        - dt -> Cycle time
    Outputs:
        - New points of delta robot
    '''
    new_poz = current_poz

    # In track robot coordinate frame
    points_X = current_poz[1]
    points_Y = current_poz[0]

    v_X = points_X * angular_velocity
    v_Y = points_Y * angular_velocity

    points_X = points_X + linear_velocity * dt + v_X * dt
    points_Y = points_Y + v_Y * dt

    new_poz[1] = points_X
    new_poz[0] = points_Y

    return new_poz

