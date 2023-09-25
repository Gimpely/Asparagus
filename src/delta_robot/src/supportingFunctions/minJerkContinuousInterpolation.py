import matplotlib.pyplot as plt
import numpy as np
import time
from numpy.core.numeric import isscalar

class minJerkContinuousInterpolation():
    def __init__(self, maxSpeed, dT=0.001, printAll=False):
        self.dT = dT
        self.maxVelocity = maxSpeed

        # define start values
        self.first_flag = False
        

    def dynamic_final_time(self, t, ref_position, current_position):
        if isscalar(ref_position):
            num_axis = 1
        else:
            num_axis = int(len(ref_position))

        # Calculate final time
        temp = abs(ref_position - current_position)
        
        if len(self.maxVelocity) > 1:
            maxVel = self.maxVelocity[:num_axis]
        
        
        temp = temp / maxVel
        if num_axis > 1:
            for idx in range(num_axis):
                if temp[idx] < 0.01:
                    temp[idx] = 0.01
                elif temp[idx] > 1:
                    temp[idx] = 1
        else:
            if temp < 0.01:
                temp = 0.01
            elif temp > 1:
                temp = 1

        #print(temp)

        tf = t + temp

        return tf

    def minJerkInterpolation(self, t, tf, x, xf):
        '''
            t .. is current time
            tf .. is final time
            x .. is current position
            xf .. is final position

            xd .. is desired position in next iteration
        '''
        if isscalar(xf):
            num_axis = 1
        else:
            num_axis = int(len(xf))

        if self.first_flag == False:
            self.xd = x
            if num_axis > 1:
                self.vel = np.zeros(num_axis)
                self.acc = np.zeros(num_axis)
                self.jerk = np.zeros(num_axis)
            else:
                self.vel = 0
                self.acc = 0

            self.first_flag = True
        
        delta_t = tf - t

        if num_axis > 1:
            for idx in range(num_axis):
                if delta_t[idx] < self.dT:
                    delta_t[idx] = self.dT
            delta_t_max = np.max(delta_t)
        else:
            if delta_t < self.dT:
                delta_t = self.dT
            delta_t_max = delta_t
        

        #print("delta_t = ", delta_t)

        delta_x = xf - self.xd

        #print("delta_x = ", delta_x)

        if num_axis > 1:
            for idx in range(num_axis):
                self.temp1 = (60*delta_x[idx]) / delta_t_max**3
                self.temp2 = (-36*self.vel[idx]) / delta_t_max**2
                self.temp3 = (-9*self.acc[idx]) / delta_t_max

                self.jerk[idx] = self.temp1 + self.temp2 + self.temp3
                # Integrate jerk to get acceleration
                self.acc[idx] = self.acc[idx] + self.dT * self.jerk[idx]
                # Integrate acceleration to get velocity
                self.vel[idx] = self.vel[idx] + self.dT * self.acc[idx]
                # Integrate velocity to get desired position
                self.xd[idx] = self.xd[idx] + self.dT * self.vel[idx]

        else:

            self.temp1 = (60*delta_x) / delta_t_max**3
            self.temp2 = (-36*self.vel) / delta_t_max**2
            self.temp3 = (-9*self.acc) / delta_t_max

            self.jerk = self.temp1 + self.temp2 + self.temp3
            # Integrate jerk to get acceleration
            self.acc = self.acc + self.dT * self.jerk
            # Integrate acceleration to get velocity
            self.vel = self.vel + self.dT * self.acc
            # Integrate velocity to get desired position
            self.xd = self.xd + self.dT * self.vel

        #print("velocity = ", self.vel)

        return self.xd

