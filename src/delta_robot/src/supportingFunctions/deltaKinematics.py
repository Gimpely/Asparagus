import numpy as np
import math
from scipy.linalg import null_space
from fractions import Fraction
import rospy


'''
    The python code was adapted from matlab code by sslajpah @ robolab, 2033
'''

def deltaInvKin(Px,Py,Pz):

    '''
        Calculates  the inverse kinematics for delta robot 3RUU and returns joint angles qq for desired TCP position P = [Px,Py,Pz]
    '''
    
    error = [False, False, False]
    qq = []
    P = np.array([[Px],[Py],[Pz+300]])

    # Robot parameters
    L1 = 250 # Upper arm (biceps) length
    L2 = 500 # Lower arm length
    r1 = 150 # Base radius
    r2 = 65 #End-effector radius

    p01 = np.array([[150], [0] ,[0]]) #Base origin to first joint
    p31 = np.array([[65], [0] ,[0]]) #TCP origin to last joint

    # Arm 1
    angle = np.radians(-60) #the angle of the first joint in respect to the origin
    R = np.matrix([[np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0,0,1]])

    
    x_local = np.transpose(R) @ P

    D = x_local + p31
    D31 = D - p01

    L2proj = np.sqrt(L2**2 - D31[1]**2) #Projection of L2 to the plane of the first joint
    Dproj = np.sqrt(D31[0]**2 + D31[2]**2) #D21projection to the plane of the first joint
    
    if np.abs((L2proj**2 - L1**2 - Dproj**2) / (-2*L1*Dproj))>1:
        rospy.loginfo("Izven obmocja arm 1")
        error[0] = True
    alpha = np.arccos(np.clip((L2proj**2 - L1**2 - Dproj**2) / (-2*L1*Dproj), -1, 1))

    gamma = np.arctan2(-D31[2],D31[0])

    theta = gamma - alpha

    qq.append(theta.item())

    # Arm 2
    angle = np.radians(60) #the angle of the first joint in respect to the origin
    R = np.matrix([[np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0,0,1]])

    x_local = np.transpose(R) @ P

    D = x_local + p31
    D31 = D - p01

    L2proj = np.sqrt(L2**2 - D31[1]**2) #Projection of L2 to the plane of the first joint
    Dproj = np.sqrt(D31[0]**2 + D31[2]**2) #D21projection to the plane of the first joint
    if np.abs((L2proj**2 - L1**2 - Dproj**2) / (-2*L1*Dproj))>1:
        rospy.loginfo("Izven obmocja arm 2")
        error[1] = True
    # alpha = np.arccos((L2proj**2 - L1**2 - Dproj**2) / (-2*L1*Dproj))
    alpha = np.arccos(np.clip((L2proj**2 - L1**2 - Dproj**2) / (-2*L1*Dproj), -1, 1))

    gamma = np.arctan2(-D31[2],D31[0])

    theta = gamma - alpha

    qq.append(theta.item())

    # Arm 3
    angle = np.radians(-180) #the angle of the first joint in respect to the origin
    R = np.matrix([[np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0,0,1]])

    x_local = np.transpose(R) @ P

    D = x_local + p31
    D31 = D - p01
    

    L2proj = np.sqrt(L2**2 - D31[1]**2) #Projection of L2 to the plane of the first joint
    Dproj = np.sqrt(D31[0]**2 + D31[2]**2) #D21projection to the plane of the first joint

    # alpha = np.arccos((L2proj**2 - L1**2 - Dproj**2) / (-2*L1*Dproj))
    if np.abs((L2proj**2 - L1**2 - Dproj**2) / (-2*L1*Dproj))>1:
        rospy.loginfo("Izven obmocja arm 3")
        error[2] = True
    alpha = np.arccos(np.clip((L2proj**2 - L1**2 - Dproj**2) / (-2*L1*Dproj), -1, 1))

    gamma = np.arctan2(-D31[2],D31[0])

    theta = gamma - alpha

    qq.append(theta.item())

    return np.array(qq), error

def rational_basis(A):

    rref, pivots = get_rref(A)

    null_space = null_space_basis(rref, pivots)

    return null_space

 

def get_rref(A):

    m, n = A.shape

    rref = A.copy()

    pivots = []

    i = 0

    for j in range(n):

        if i >= m:

            break

        pivot_row = i

        while abs(rref[pivot_row, j]) < 1e-8:

            pivot_row += 1

            if pivot_row == m:

                pivot_row = i

                j += 1

                if j == n:

                    break

        if j == n:

            break

        pivots.append(j)

        if pivot_row != i:

            rref[[i, pivot_row], j:] = rref[[pivot_row, i], j:]

        pivot = rref[i, j]

        rref[i, j:] = rref[i, j:] / pivot

        for k in range(m):

            if k == i:

                continue

            factor = rref[k, j] / rref[i, j]

            rref[k, j:] -= factor * rref[i, j:]

        i += 1

    return rref, pivots

 

def null_space_basis(rref, pivots):

    m, n = rref.shape

    basis = []

    free_columns = [j for j in range(n) if j not in pivots]

    for j in free_columns:

        vec = np.zeros(n)

        vec[j] = 1

        for i in reversed(pivots):

            vec[i] = -rref[i, j]

            for k in range(i + 1, m):

                vec[i] -= rref[i, k] * vec[k]

        basis.append(vec)

    return np.array(basis).T


def deltaIForwardKin(qq):
    '''
    Returns the forward kinematics for delta robot 3RUU, calculates TCP position for given joint angles qq
    '''
     # Robot parameters
    L1 = 250 # Upper arm (biceps) length
    L2 = 500 # Lower arm length
    r1 = 150 # Base radius
    r2 = 65 #End-effector radius

    #Sphere centers

    #Arm 1
    angle = np.radians(-60) #the angle of the first joint in respect to the origin
    R = np.matrix([[np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0,0,1]])
    
    j2_1 = R @ (np.array([r1, 0, 0]) + L1 * np.array([np.cos(-qq[0]), 0, np.sin(-qq[0])]) - np.array([r2, 0, 0]))
    # print("j21: ", j2_1)
    
    #Arm 2
    angle = np.radians(60) #the angle of the first joint in respect to the origin
    R = np.matrix([[np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0,0,1]])
    
    j2_2 = R @ (np.array([r1, 0, 0]) + L1 * np.array([np.cos(-qq[1]), 0, np.sin(-qq[1])]) - np.array([r2, 0, 0]))
    # print("j22: ", j2_2)
    #Arm 3
    angle = np.radians(-180) #the angle of the first joint in respect to the origin
    R = np.matrix([[np.cos(angle), -np.sin(angle), 0],
                    [np.sin(angle), np.cos(angle), 0],
                    [0,0,1]])
    
    j2_3 = R @ (np.array([r1, 0, 0]) + L1 * np.array([np.cos(-qq[2]), 0, np.sin(-qq[2])]) - np.array([r2, 0, 0]))
    # print("j23: ", j2_3)

    '''
    The solution adapted form (link)
    '''

    #Rewriting points

    #Rewriting points

    x1 = j2_1.flat[0]
    y1 = j2_1.flat[1]
    z1 = j2_1.flat[2]
  
    x2 = j2_2.flat[0]
    y2 = j2_2.flat[1]
    z2 = j2_2.flat[2]

    x3 = j2_3.flat[0]
    y3 = j2_3.flat[1]
    z3 = j2_3.flat[2]


    # A*x - b = 0
    # x = [x²+y²+z²; x; y; z]

    A = np.matrix([[1, -2*x1, -2*y1, -2*z1],
                   [1, -2*x2, -2*y2, -2*z2],
                   [1, -2*x3, -2*y3, -2*z3]])

    b = np.matrix([[L2**2-x1**2-y1**2-z1**2],
                   [L2**2-x2**2-y2**2-z2**2],
                   [L2**2-x3**2-y3**2-z3**2]])
    
    
    # print("b:", b)

    # Solution x = xp (partial solution) + t * z

    #Xp, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    Xp = np.dot(np.linalg.pinv(A), b)
    xp = Xp[1:4]     

    Z = rational_basis(A)

    z = Z[1:4, :]
    # print("z",z)
    # print("--------")

    a2 = z[0]**2 + z[1]**2 + z[2]**2
    a1 = 2*(z[0]*xp[0] + z[1]*xp[1] + z[2]*xp[2]) - Z[0]
    a0 = xp[0]**2 + xp[1]**2 + xp[2]**2 - Xp[0]

    a2 = a2.item()
    a1 = a1.item()
    a0 = a0.item()

    p = np.array([a2, a1, a0])
    t = np.roots(p)
    
    N1 = xp + t[0] * z
    N2 = xp + t[1] * z

    if N1[2] < N2[2]:
        xx = N1
    else:
        xx = N2
    xx[2] = xx[2] - 300
    P = xx
    
    return P

if __name__ == '__main__':
    x = 0
    y = 0
    z = -671

    qq = deltaInvKin(x,y,z)
    print(qq)

    P=deltaIForwardKin(qq)
    print(P)

