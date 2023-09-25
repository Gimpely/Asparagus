import numpy as np
import cv2

import matplotlib.pyplot as plt


'''
	Erik 24.6.2021
'''

def simulateLaser(showSimulated=False):
	global minAngle
	global maxAngle

	phi = np.pi/2-0.3
	phi_increment = 0.003
	R = 300

	N_asparagus = 4
	N = 20
	scanPoints = []
	scanAngles = []
	for j in range(N_asparagus):
		for i in range(N):
			r = R*(1 - 0.1*np.sin(2*np.pi*0.5*i/N))
			scanPoints.append(r)

			scanAngles.append(phi)
			phi += phi_increment
		

	minAngle = scanAngles[0]
	maxAngle = scanAngles[-1]

	if showSimulated:
		scanPoints = np.array(scanPoints)
		scanAngles = np.array(scanAngles)

		x = scanPoints*np.cos(scanAngles)
		y = scanPoints*np.sin(scanAngles)

		plt.plot(x, y, 'o')
		plt.title('Simulated points')
		plt.gca().set_aspect('equal', adjustable='box')
		plt.show()
		
	return scanPoints, phi_increment


# --------------------------------------------

def transform_ScanToGlobal(points, 
				T_o=[377.36018898,  293.9326781],
				T_x=[376.4866538,   495.11714984],

				resultCS='global'
				):
	''' 
		points - can be one or multiple poins in an array.

		resultCS - 'global' or 'scan' (direction of transformation)
	'''
	# Points of origin of global cs and of point along x axis
	# T_o = [-100, 500]
	# T_x = [-200, 500]

	T_o = np.array(T_o)
	T_x = np.array(T_x)


	def R(phi):
		return np.array([[np.cos(phi), -np.sin(phi)],[np.sin(phi), np.cos(phi)]])

	def toHomo(vec):
		homoVec = np.ones((vec.shape[0]+1))
		homoVec[:vec.shape[0]] = vec
		return homoVec.T


	x_direction = T_x - T_o
	phi = np.arctan2(x_direction[1], x_direction[0])
	d = T_o


	H_g_s = np.zeros((3,3))
	H_g_s[2, 2] = 1
	H_g_s[:2, :2] = R(phi)
	H_g_s[:2, 2] = T_o


	# H scan relative to Global ( xG = H_s_g * xS)
	H_s_g = np.linalg.inv(H_g_s)

	if False:
		# Testing and debug
		print(H_g_s)
		print('inverse:')
		print(H_s_g)

		T_1 = np.array([0, 0])
		T_2 = np.array([100, 0])

		print('try from scan')
		# print(toHomo(T_o))

		print(np.dot(H_s_g, toHomo(T_o)))
		print(np.dot(H_s_g, toHomo(T_x)))
		print('try from glob')
		print(np.dot(H_g_s, toHomo(T_1)))
		print(np.dot(H_g_s, toHomo(T_2)))


	if resultCS == 'global':
		H = H_s_g
	elif resultCS == 'scan':
		H = H_g_s
	else:
		print('Invalid CS in transform_ScanToGlobal.')
		return -1

	# compute points:
	points = np.array(points)
	if len(points.shape) > 1:
		transformedPoints = []
		for point in points:
			transformedPoints.append(np.dot(H, toHomo(point)))
		return np.array(transformedPoints)[:,:2]

	else:
		return np.dot(H, toHomo(points))[:2]


def detectAsparagus(distances, phi_increment=None, minAngle=0, maxAngle=1.5,
				xLim=[-500, 500],
				yLim=[-500, 500],
				phiOffset=0,
				plotResults=False, printAll=False):
	'''
	'''
	# ### Parameters: ###
	R_asparagus = 8

	# In laser coordinates:



	# printAll = False
	# plotResults = True

	# Get correct data from subscription,....
	# TODO:


	# ____________________________________________________________________
	# - Detection:
	# prepare data:
	distances = np.array(distances)
	scanAngles = np.linspace(minAngle, maxAngle, distances.shape[0]) + phiOffset

	# compute:
	x = []
	y = []
	for i in range(distances.shape[0]):
		new_x = distances[i]*np.cos(scanAngles[i]) 
		new_y = distances[i]*np.sin(scanAngles[i]) 

		if (xLim[0] < new_x < xLim[1]) and (yLim[0] < new_y < yLim[1]):
			x.append(new_x)
			y.append(new_y)

	x = np.array(x)
	y = np.array(y)
	xy = np.vstack((x,y)).T



	# detect:
	pointsGroups = []
	pointsGroup = []
	for i in range(1, x.shape[0]):
		dist = np.linalg.norm([[x[i]-x[i-1]], [y[i]-y[i-1]]])
		# print(dist)
		if dist < R_asparagus:
			pointsGroup.append(xy[i-1])
		elif len(pointsGroup) >= 2:	# We want at least two points
			pointsGroups.append(pointsGroup)
			pointsGroup = []
	if len(pointsGroup) >= 2:	# We want at least two points
		pointsGroups.append(pointsGroup)

	if printAll:
		print('groups:')
		print(pointsGroups)

	asparagusPoints = []
	for group in pointsGroups:
		group = np.array(group)
		asparagusPoints.append(np.average(group, axis=0))

	asparagusPoints = np.array(asparagusPoints)


	if printAll:
		print('asparagus Points:')
		print(asparagusPoints)


	# Results printing:
	if plotResults:

		# Used points:
		for group in pointsGroups:
			group = np.array(group)
			plt.plot(group[:,0], group[:,1], 'o')
		
		plt.plot(asparagusPoints[:,0], asparagusPoints[:,1], ':gx', markersize=10)

		plt.plot(xy[:,0], xy[:,1], '.')

		plt.gca().set_aspect('equal', adjustable='box')
		plt.show()

	return asparagusPoints, xy


def generateHarvestPoints(asparagusPoint, approachAngle=np.pi/2, gOpen=-110, gClosed=0):
	'''
		asparagusPoint (in 2D)
	'''
	
	harvestHeight = 0
	approachHeight = 100
	approachDistance = 50
	approachDistanceFar = 150

	xAsparagus = asparagusPoint[0]
	yAsparagus = asparagusPoint[1]

	xApproach = xAsparagus + approachDistance*np.cos(approachAngle)
	yApproach = yAsparagus + approachDistance*np.sin(approachAngle)
	xApproachFar = xAsparagus + approachDistanceFar*np.cos(approachAngle)
	yApproachFar = yAsparagus + approachDistanceFar*np.sin(approachAngle)

	toolOrientation = 180/np.pi*(np.pi - approachAngle)	# command for the robot

	gP = []
	# approach Far:
	gP.append(np.array([xApproachFar, yApproachFar, approachHeight, toolOrientation, gOpen]))
	# approach Closer:
	gP.append(np.array([xApproach, yApproach, approachHeight, toolOrientation, gOpen]))
	# Lower:
	gP.append(np.array([xApproach, yApproach, harvestHeight, toolOrientation, gOpen]))

	# approach Lower:
	gP.append(np.array([xAsparagus, yAsparagus, harvestHeight, toolOrientation, gOpen]))

	# cut:
	waitTime = [-1 for i in gP]
	gP.append(np.array([xAsparagus, yAsparagus, harvestHeight, toolOrientation, gClosed]))
	waitTime.append(0.5)

	# raise:
	# -- Raise straight up:
	# gP.append(np.array([xAsparagus, yAsparagus, approachHeight, toolOrientation, gClosed]))
	# -- Raise to and move away
	gP.append(np.array([xApproachFar, yApproachFar, approachHeight, toolOrientation, gClosed]))
	waitTime.append(-1)

	return gP, waitTime


def generateDepositPoints(depositPoint, gOpen=-110, gClosed=0, approachAngle=np.pi):
	depositHeight = 60
	approachHeight = 60
	approachDistance = 50

	toolOrientation = 180/np.pi*(np.pi - approachAngle) # command for the robot

	
	xAsparagus = depositPoint[0]
	yAsparagus = depositPoint[1]

	xApproach = xAsparagus + approachDistance*np.cos(approachAngle)
	yApproach = yAsparagus + approachDistance*np.sin(approachAngle)
	
	gP = []

	# approach Closer:
	gP.append(np.array([xAsparagus, yAsparagus, approachHeight, toolOrientation, gClosed]))
	# Lower:
	# gP.append(np.array([xAsparagus, yAsparagus, depositHeight, toolOrientation, gClosed]))


	# release:
	# gP.append(np.array([xAsparagus, yAsparagus, depositHeight, toolOrientation, gOpen]))

	waitTime = [-1 for i in gP]
	gP.append(np.array([xAsparagus, yAsparagus, approachHeight, toolOrientation, gOpen]))
	waitTime.append(1)

	
	gP.append(np.array([xApproach, yApproach, approachHeight, toolOrientation, gOpen]))
	waitTime.append(-1)

	return gP, waitTime



# --------------------------------------------------------------------------------------
# Testing the code:

# # Test detection:
# distances, phi_increment = simulateLaser(showSimulated=0)
# asparagusPoints, allUsedPoints = detectAsparagus(distances, plotResults=True)
# print(asparagusPoints)


# Test transformation:
# points = [
# 	[0, 0],
# 	[10, 0],
# 	[0, 10],
# ]
# pT = transform_ScanToGlobal(points, resultCS='scan')

# print(pT)

# pT = transform_ScanToGlobal(pT, resultCS='global')

# print(pT)