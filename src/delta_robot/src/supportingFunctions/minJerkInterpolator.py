'''
	Copied from: https://mika-s.github.io/python/control-theory/trajectory-generation/2017/12/06/trajectory-generation-with-a-minimum-jerk-trajectory.html
	on 21.6.2021

	And then modified by Erik P
'''


import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from numpy.core.numeric import isscalar



class minJerkInterpolator():


	def __init__(self, maxSpeed, dT=0.001, printAll=False):
		'''
			maxSpeed can be scalar (if it's equal for all dimensions), or numpy.ndarray, if every dimension has different maxSpeed

			dT is used only if current time is not used ()
		'''

		if isscalar(maxSpeed):
			self.maxSpeed = np.array(maxSpeed)
			self.maxSpeed.shape = (1)
		else:
			self.maxSpeed = np.array(maxSpeed)


		self.dT = dT
		self.printAll = printAll
		self.pointsFifo = []		# List of dictionares for points in shape [ {'point': point, 'speed': maxVelocity}), ...]
		self.timeStartedMove = time.time()
		self.timeEstMoveComplete = time.time()
		self.timerWaitTime = time.time()
		self.startedWaiting = False

	def addPoint(self, point, maxVelocity=None, waitAfter=-1):
		''' waitAfter - seconds to wait after continuing with the next point.'''
		if maxVelocity is None:
			maxVelocity = self.maxSpeed

		if len(self.pointsFifo) == 1:
			self.timeStartedMove = time.time()
		if self.printAll:
			print('Point added to fifo:', point)
		self.pointsFifo.append({'point': np.copy(point), 'maxSpeed': maxVelocity, 'waitAfter': waitAfter})

	def clearPoints(self):
		''' Clears all points in fifo except the current move points
		'''
		self.pointsFifo = self.pointsFifo[:2]

	def printPoints(self):
		print('Current {} points in Fifo:'.format(len(self.pointsFifo)))
		for point in self.pointsFifo:
			print(point)

	def run(self, robot_in_poz=False):
		'''	
			call to get interpolation point (and it takes care of interpolation 
					and of selecting the next point, and removing old points)

			return: status, interpolatedPoint
				status - 	0 if there is no interpolation points
								1 only one point (this is return of the interpolation)
								2 two or more interpolation points, interpolation in progress
				interpolatedPoint - self exp.
		'''
		self.robot_in_poz = robot_in_poz

		if len(self.pointsFifo) == 0:
			return 0, np.array([0])
		elif len(self.pointsFifo) == 1:
			return 1, self.pointsFifo[0]['point']
		else:
			# interpolation:

			timeSinceMoveStart = time.time() - self.timeStartedMove
			goalReached, intValues = self.interpolation(self.pointsFifo[0]['point'], 
									 self.pointsFifo[1]['point'], 
									 t=timeSinceMoveStart,
									 maxVelocity=self.pointsFifo[1]['maxSpeed'])
			
			#print("Goal reached = ", goalReached)
			if goalReached or self.robot_in_poz:
				continueToNext = False

				if self.pointsFifo[1]['waitAfter'] > 0:
					if self.startedWaiting:
						if time.time() - self.timerWaitTime > self.pointsFifo[1]['waitAfter']:
							continueToNext = True
					else:
						self.timerWaitTime = time.time()
						self.startedWaiting = True
				else:
					continueToNext = True
				
				if continueToNext:
					self.pointsFifo.pop(0)
					self.timeStartedMove = time.time()
					self.startedWaiting = False
			
			return 2, intValues



	def interpolation(self, starting, final, t, maxVelocity=None):
		'''
			interpolation of n dimensions
			return: goalReached (0 or 1), interpolated point (vector)
		'''
		if maxVelocity is None:
			maxVelocity = self.maxSpeed

		onlyOneDimension = False

		if np.isscalar(starting):
			onlyOneDimension = True

		starting = np.array(starting)
		final = np.array(final)

		#print("starting = ", starting)
		#print("final = ", final)

		if onlyOneDimension:
			starting.shape = (1)
			final.shape = (1)

		distances = np.abs(final - starting)
		if maxVelocity.shape[0] <= 1 and distances.shape[0] > 1:
			maxVelocity = maxVelocity*np.ones(distances.shape)

		#print(distances)	
		
		if np.max(distances) < 1e-5:
			goalReached = True
			#print("goal_reached = ", goalReached)
			return goalReached, starting
		
		times = 1.0*distances/maxVelocity

		# Which dimension will take the longest:
		idx = np.argmax(times)

		if self.printAll:
			print('times:', times)
			print('idx', idx)



		goalReached, point, pointDerivative = self.generateMinJerk(0, abs(final[idx]-starting[idx]), self.dT, maxVelocity[idx], t)
		#goalReached, point, pointDerivative = self.generateMinJerk(starting[idx], final[idx], self.dT, maxVelocity[idx], t)
		refPoint = point/distances[idx]

		interpolated = starting + (final-starting)*refPoint

		if onlyOneDimension:
			interpolated = interpolated[0]

		return goalReached, interpolated


	def generateMinJerk(self, current, setpoint, dT, maxVelocity , t=None):
		'''
			Generates min jerk trajectory.

			if t is None generates whole trajectory. if t is skalar, then only point at time t is generated (0<t<move_time)

			params:
			current - position
			setpoint - desired position
			dT - time step between points
			maxVelocity
			t - (if not None it returns only the position at this time)

			returns:
			goalReached (false if t=None, if t is passed, goalReached = True, when t> move_time)
		'''
		#print("setpoint = ", setpoint)
		if abs(setpoint - current) < 1: #1e-5:
			goalReached = True
			return goalReached, current, 0
		
		move_time = 1.1 * abs(setpoint - current) / maxVelocity
		goalReached = False

		if t is not None:
			if 0 <= t:
				time = t
				if t >= move_time:
					goalReached = True
					time = move_time
			else:
				print('Time must not be negative!')
				return -1
		else:
			time = np.arange(0, move_time, dT)


		# if self.printAll:
		# 	print('time / Move time / goalReached:', t, '/', move_time, '/', goalReached) 



		trajectory = current + (setpoint - current) * (10.0 * (time/move_time)**3 
				- 15.0 * (time/move_time)**4
				+ 6.0 * (time/move_time)**5)

		trajectory_derivative = (1.0/move_time) * (setpoint - current) * (30.0 * (time/move_time)**2.0
				- 60.0 * (time/move_time)**3.0
				+ 30.0 * (time/move_time)**4.0)

		return goalReached, trajectory, trajectory_derivative



if __name__ == "__main__":


	# Set up and calculate trajectory.

	maxSpeed = 200
	frequency = 10
	dT = 1/frequency
	interpolator = minJerkInterpolator(maxSpeed=maxSpeed, dT=dT, printAll=False)


	x1 = [10, 10, 10]
	x2 = [100, 10, 10]
	x3 = [100, 100, 10]
	x4 = [10, 100, 10]
	x5 = [10, 10, 10]





	# # Create plot.
	# xaxis = np.arange(0, dT*traj.shape[0], 1/frequency)

	# plt.plot(xaxis, traj)
	# plt.plot(xaxis, traj_vel)
	# plt.title("Minimum jerk trajectory")
	# plt.xlabel("Time [s]")
	# plt.ylabel("Angle [deg] and angular velocity [deg/s]")
	# plt.legend(['pos', 'vel'])

	# plt.show()


	tStart = time.time()
	image = np.zeros((700,1023,3), np.uint8)





	interpolator.addPoint(x1)
	interpolator.addPoint(x2)
	interpolator.addPoint(x3)

	interpolator.printPoints()


	x = 10 
	y = 10
	z = 10

	frameTime = 100

	while True:

		interpolator.printPoints()

		start = [200, 200, 100]
		stop = [800, 250, 5]

		intStatus, intermediate = interpolator.run()

		if intStatus != 0:
			x = intermediate[0]
			y = intermediate[1]
			z = intermediate[2]

		image = np.zeros((700,1023,3), np.uint8)

		cv2.circle(image,(int(x),int(y)), int(z), (0,255,0), -1)
		# cv2.circle(image,(int(x),int(y)), int(z), (50,50,50), -1)
			
			
		cv2.imshow('Okno', image)


		k = cv2.waitKey(frameTime) & 0xFF
		if k == ord('q'):
			break
		elif k == ord('r'):
			image = np.zeros((700,1023,3), np.uint8)
			tStart = time.time()
		elif k == ord('a'):
			interpolator.addPoint(x4)
			interpolator.addPoint(x5)

			





	cv2.destroyAllWindows()
