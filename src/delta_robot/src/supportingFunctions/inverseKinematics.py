

import numpy as np 
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D





# ### Supporting functions: ###
def rotZ(psi, r):
	'''
		Rotates vector r around z-axis for angle psi
	'''
	x = r[0]*np.cos(psi) - r[1]*np.sin(psi)
	y = r[0]*np.sin(psi) + r[1]*np.cos(psi)
	z = r[2]
	rRotated = [x, y, z]
	return rRotated


def vecSum(r1, r2):
	p = [0, 0, 0]
	p[0] = r1[0] + r2[0]
	p[1] = r1[1] + r2[1]
	p[2] = r1[2] + r2[2]
	return p


def vecDiff(r1, r2):
	p = [0, 0, 0]
	p[0] = r1[0] - r2[0]
	p[1] = r1[1] - r2[1]
	p[2] = r1[2] - r2[2]
	return p


# ### Main function: ###
def deltaInverseKin(Tx, Ty, Tz):
	''' 
		Computes inverse kinematics for a rotational joints delta robot
		distance units are mm, angle units radians.
	
	'''
	# ### Robot parameters and dimensions: ###
	# TODO: enter the correct values from CAD (current values are approximate)
	l_1 = 315   # length of link 1 (distance between joint axes)
	l_2 = 605   # length of link 2

	# -- r_0:  (end of 'link 0' = base. or Position of joint 1 in base system)
	r_0x = 170  # 
	r_0y = 0    #  <-- this one should stay zero
	r_0z = 30  	#
	r_0 = [r_0x, r_0y, r_0z]	# don't need this one (just for debug)

	# -- s_3: (s_3 = - r_3   [r_3 is from joint 3 to end of link 3. Platform end relative to joint 3)
	s_3x = 80   #S
	s_3y = 0    # <-- this one should stay zero
	s_3z = 10   #
	s_3 = [s_3x, s_3y, s_3z]

	# -- angles:	(angles of the motors - 3 motors in 2pi)
	armAngles = [0, 2*np.pi/3, 2*np.pi*2/3]



	# ### Function code: ### 
	alphas = np.zeros((3))

	r_pb = [Tx, Ty, Tz]	# r of platform relative to base
	if False:
		print('Distance from base to desired platform position:')
		print(np.linalg.norm(r_pb))

	# print('Debug -------------------\n-------------------')

	for i in range(3):
		p_3 = rotZ(-armAngles[i], r_pb)

		p_2 = vecSum(p_3, s_3)

		# From joint 1 to joint 3
		Dx = p_2[0] - r_0x
		Dy = p_2[1] - r_0y
		Dz = p_2[2] - r_0z

		# print('i =', i, '  #--#--#--#--#--#--#--#--#--#--')
		# print('Dx = ', Dx)
		# print('Dy = ', Dy)
		# print('Dz = ', Dz)

		d_2 = np.sqrt(l_2**2 - Dy**2)
		a = np.sqrt(Dx**2 + Dz**2)

		# print(f'i = {i} - Length of vector D:')
		# print(np.linalg.norm([Dx, Dy, Dz]))

		cosDelta = (l_1**2 + a**2 - d_2**2)/(2*l_1*a)
		if cosDelta**2 > 1:
			print('Position not reachable')
			alpha = -3
		else:
			delta = np.arccos(cosDelta)
			gamma = np.arctan2(-Dz, Dx)
			alpha = gamma - delta
		alphas[i] = alpha

	return [alphas, [armAngles, l_1, l_2, r_0, s_3]]


# ### Testing functions:

def plot3DLine(p1, p2, ax, c='b', marker='o'):
	if type(p1) == int or type(p1) == float:
		p1 = [p1 for i in range(3)]
	xPoints = [p1[0], p2[0]]
	yPoints = [p1[1], p2[1]]
	zPoints = [p1[2], p2[2]]
	ax.plot(xPoints, yPoints, zPoints, c=c, marker=marker)


def testInverseKin(r_pb, alphas, otherArgs):
	armAngles, l_1, l_2, r_0, s_3 = otherArgs
	print('testInverseKin run')

	# print('Testing lengths of links:')
	# for i in range(3):
	# 	cLink2 = (r_pb + s_3) - (r_0 + [l_1*np.cos()])


	# 3D Plot (of all points)
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	cubeSize = 400
	zTop = 100
	ax.set_xlim3d(-cubeSize, cubeSize)
	ax.set_ylim3d(-cubeSize, cubeSize)
	ax.set_zlim3d(zTop-2*cubeSize, zTop)

	# plot:
	for i in range(3):
		# Base
		link0End = rotZ(armAngles[i], r_0)
		plot3DLine(0, link0End, ax, c='k')

		# link 1
		link1End = vecSum(link0End, rotZ(armAngles[i], [l_1*np.cos(alphas[i]), 0, -l_1*np.sin(alphas[i])]))
		plot3DLine(link0End, link1End, ax, c='b')

		# link 2
		link2End = vecSum(r_pb, rotZ(armAngles[i], s_3))
		plot3DLine(link1End, link2End, ax, c='g')

		# platform
		plot3DLine(link2End, r_pb, ax, c='y')

		# print(f'Arm {i} Link1 length:', np.linalg.norm(vecDiff(link1End, link0End)))
		print('Arm {} Link2 length:'.format(i), np.linalg.norm(vecDiff(link2End, link1End)))
		
		

	plt.title('Resitev inverzne kinematike')
	ax.set_xlabel('X [mm]')
	ax.set_ylabel('Y [mm]')
	ax.set_zlabel('Z [mm]')
	# plt.savefig(directory + fileName + 'ZGLED.png', dpi = 200)

	# ######
	# TODO: here you have the image rotatig:
	# for angle in range(0, 360):
	# 	ax.view_init(30, angle)
	# 	plt.draw()
	# 	plt.pause(.001)

	ax.view_init(30, 15)

	import sys
	directory = sys.path[0]
	plt.savefig(directory + '/deltaIK4.png', dpi=200)
	plt.show()






# testVec = [np.cos(np.pi/180*120), np.sin(np.pi/180*120), 0]
# testVec = [0, 1, 0]
# print('input vec:\n', testVec)
# print(rotZ(-np.pi/2, testVec))



if __name__ == "__main__":

	# Main code:

	# rpb = [100, 0, -500]
	vectors = []
	# rpb = [200, 0, -600]
	# vectors.append(rpb)
	# rpb = [200*np.cos(2*np.pi/3), 200*np.sin(2*np.pi/3), -600]
	# vectors.append(rpb)
	# rpb = [200*np.cos(2*np.pi*2/3), 200*np.sin(2*np.pi*2/3), -600]
	rpb = [300, 0, -500]
	rpb = [0, 0, -500]
	rpb = [0, 0, -800]
	rpb = [0, 300, -500]
	rpb = [400, -300, -300]
	vectors.append(rpb)

	solutions = []
	for rpb in vectors:
		rx = rpb[0]
		ry = rpb[1]
		rz = rpb[2]
		print('-------------- Starting function:')
		alphas, other = deltaInverseKin(rx, ry, rz)
		print('-------------- function ended:')
		print('Angles:', alphas)

		# Test inverse kin:
		testInverseKin(rpb, alphas, other)
		plt.pause(0.01)
		plt.clf()
		solutions.append(alphas)

	print('alphas: ')
	for alphas in solutions:
		print(alphas)












