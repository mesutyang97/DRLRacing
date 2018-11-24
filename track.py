# track.py
# This file contains all the necessary component to simulate a ractrack

import numpy as np
import math
import utils

# Position: not location, but where you
# tInput: throttle input
# sInput: steering input: -1: left. +1: right

STEPT = 0.1

COLLISONPEN = 1.0

class Track:
	def __init__(self, miu, dot_miu, w_feet, l_feet, board_lst, dot_lst, finish_line):
		# Friction
		self.miu = miu
		w = utils.ftToMm(w_feet)
		l = utils.ftToMm(l_feet)
		self._grid = self.initializeGrid(w, l)
		self.buildDots(dot_lst, w, l)
		self.buildBoards(board_lst)
		self.buildFinishLine(finish_line)


	'''
	Initialize a 1-padded zero arrays as the grid
	'''
	def initializeGrid(self, w, l):
		g = np.zeros((w, l))
		# Zero pad the boarders
		g[0:100,:] = np.ones((100, l))
		g[w-100:w,:] = np.ones((100, l))
		g[:,0:100] = np.ones((w, 100))
		g[:,l-100:l] = np.ones((w, 100))
		return g.copy()

	'''
	The board_lst will be a list of form
	((endpoint_A_y, endpoint_A_x), (endpoint_B_y, endpoint_B_x), width)
	Enforced: endpoint_A_y <= endpoint_B_y, and endpoint_A_x <= endpoint_B_x
	'''
	def buildBoards(self, board_lst):
		for board in board_lst:
			endpoint_A_y_ft, endpoint_A_x_ft = board[0]
			endpoint_A_y = utils.ftToMm(endpoint_A_y_ft)
			endpoint_A_x = utils.ftToMm(endpoint_A_x_ft)

			endpoint_B_y_ft, endpoint_B_x_ft = board[1]
			endpoint_B_y = utils.ftToMm(endpoint_B_y_ft)
			endpoint_B_x = utils.ftToMm(endpoint_B_x_ft)
			assert endpoint_A_y <= endpoint_B_y, "Endpoint invariant in y violated"
			assert endpoint_A_x <= endpoint_B_x, "Endpoint invariant in x violated"

			half_width = utils.ftToMm(board[2]/2)
			# Board is horizontal, or, aligned with length
			if endpoint_A_y == endpoint_B_y:
				self._grid[endpoint_A_y - half_width:endpoint_A_y + half_width, endpoint_A_x: endpoint_B_x] = np.ones(((2*half_width), endpoint_B_x- endpoint_A_x))
			# Board is vertical, or, aligned with width
			elif endpoint_A_x == endpoint_B_x:
				self._grid[endpoint_A_y: endpoint_B_y, endpoint_A_x - half_width:endpoint_A_x + half_width] = np.ones((endpoint_B_y- endpoint_A_y, (2*half_width)))
			# No support of diagonal board yet
			else:
				print("Error: do not support diagonal board")

	'''
	The dot_lst will be a list of form
	((y,x), radius), in feets
	'''
	def buildDots(self, dot_lst, w, l):
		for dot in dot_lst:
			y_ft, x_ft = dot[0]
			y = utils.ftToMm(y_ft)
			x = utils.ftToMm(x_ft)

			r_ft = dot[1]
			r = utils.ftToMm(r_ft)

			a,b = np.ogrid[-y:w-y, -x:l-x]
			mask = a*a + b*b <= r*r

			self._grid[mask] = -1
	

	'''
	The finish line will be in the form of
	((endpoint_A_y, endpoint_A_x), (endpoint_B_y, endpoint_B_x), width)
	'''
	def buildFinishLine(self, finish_line):
		endpoint_A_y_ft, endpoint_A_x_ft = finish_line[0]
		endpoint_A_y = utils.ftToMm(endpoint_A_y_ft)
		endpoint_A_x = utils.ftToMm(endpoint_A_x_ft)

		endpoint_B_y_ft, endpoint_B_x_ft = finish_line[1]
		endpoint_B_y = utils.ftToMm(endpoint_B_y_ft)
		endpoint_B_x = utils.ftToMm(endpoint_B_x_ft)
		assert endpoint_A_y <= endpoint_B_y, "Endpoint invariant in y violated"
		assert endpoint_A_x <= endpoint_B_x, "Endpoint invariant in x violated"

		half_width = utils.ftToMm(finish_line[2]/2)
		# Board is horizontal, or, aligned with length
		if endpoint_A_y == endpoint_B_y:
			self._grid[endpoint_A_y - half_width:endpoint_A_y + half_width, endpoint_A_x: endpoint_B_x] = -2 * np.ones(((2*half_width), endpoint_B_x- endpoint_A_x))
		# Board is vertical, or, aligned with width
		elif endpoint_A_x == endpoint_B_x:
			self._grid[endpoint_A_y: endpoint_B_y, endpoint_A_x - half_width:endpoint_A_x + half_width] = -2 * np.ones((endpoint_B_y- endpoint_A_y, (2*half_width)))
		# No support of diagonal board yet
		else:
			print("Error: do not support diagonal board")


	def getGrid(self):
		return self._grid


class Car:
	def __init__(self, startingState, carNumber, startingRanking, mass, length, width):
		self.carNumber = carNumber
		self._mass = mass
		self.a = a



class CarState:
	def __init__(self, startLocation, startVelocity, startRanking, mass, drag, topSpeed, maxTurningAngle, length, width, isLinear = True):
		self._location = startLocation
		self._velocity = startVelocity
		self._rank = startRanking
		self._mass = mass
		self._drag = drag
		self._steering = Steering(maxTurningAngle)
		self._throttle = Throttle(drag, topSpeed, isLinear)
		self._length = length
		# self._cornerDist = np.norm([length, width])

	def getLocation(self):
		return self._location

	def getVelocity(self):
		return self._velocity

	def checkCollison(self, nextVelocity_unit, desired_nextVelocity, desired_nextLocation, curTrack):
		headPosition = nextVelocity_unit * (self._length/2)
		x = int(headPosition[0])
		y = int(headPosition[1])
		if curTrack.grid[x][y]:
			return (0, 0), self._location - COLLISONPEN* desired_nextVelocity
		else:
			return desired_nextVelocity, desired_nextLocation



	def step(self, sInput, tInput, curTrack):
		# The current track contains information about other car on the track
		
		
		curSpeed = np.norm(self._velocity)
		curVelocity_unit = self._velocity/curSpeed
		# Assertion
		assert np.norm(velocity_unit) == 1

		a_lim = curTrack.miu * 9.8
		turningAngle, a_c = self._steering.getAC(curSpeed, sInput, a_lim)
		# This step only changed orientation, not magnitude
		nextVelocity_unit = self._steeing.rotateVelocity(curVelocity_unit, turningAngle)
		a_lim_t = math.sqrt(np.squre(a_lim) - np.square(a_c))
		nextSpeed = self._throttle.getNewSpeed(curSpeed, tInput, a_lim_t)
		desired_nextVelocity = nextSpeed * nextVelocity_unit
		desired_nextLocation = self._location + self._velocity * STEPT
		self._velocity, self._location = checkCollison(nextVelocity_unit, desired_nextVelocity, desired_nextLocation)


class Steering:
	def __init__(self, maxTurningAngle):
		self._maxAngle = maxTurningAngle

	def getAC(self, curSpeed, sInput, a_lim):
		assert(sInput <= 1.0 and sInput >= -1.0)
		# Get the centripital component of acceleration
		assumedTravel = curSpeed * STEPT
		turningAngle = sInput * self._maxAngle
		# This come from a drawing
		turningR = abs(assumedTravel / (2.0 * math.sin(math.radians(turningAngle))))
		a_c = curSpeed * curSpeed / turningR
		if a_c < a_lim:
			return turningAngle, a_c
		else:
			turningR_max = curSpeed * curSpeed / a_lim
			turningAngle_max = math.degrees(math.asin(assumedTravel / (2.0 * turningR_max))) * sInput / abs(sInput)
			return turningAngle_max, a_lim
	'''
	def rotateVelocity(self, curVelocity_unit, turningAngle):
		assert turningAngle < self._maxAngle
		old_theta = 0
		if curVelocity_unit[0] == 0:
			if curVelocity_unit[1] > 0:
				print("used 1")
				old_theta = 1.5707963267948966
			else:
				print("used 2")
				old_theta = - 1.5707963267948966
		else:
			old_theta = math.tan(curVelocity_unit[1]/curVelocity_unit[0])
		
		new_theta = old_theta - math.radians(turningAngle)

		# checkme
		return (math.cos(new_theta), math.sin(new_theta))
	'''
	def rotateVelocity(self, curVelocity_unit, turningAngle):
		return utils.rotateVector(curVelocity_unit, - turningAngle)


class Throttle:
	def __init__(self, drag, topSpeed, isLinear = True):
		self._drag = drag
		self._lim_a = lim_a
		self._topSpeed = topSpeed
		self._isLinear = isLinear

	def getNewSpeed(self, curSpeed, tInput, a_lim_t):
		assert(tInput <= 1.0 and tInput >= -1.0)

		if self._isLinear:
			# Coasting: just drag
			if (tInput == 0):
				return curSpeed * (1 - self._drag)

			desiredNextSpeed = self._topSpeed * tInput
			desired_a_t = (desiredNextSpeed - curSpeed)/STEPT
			a_t = desired_a_t
			if desired_a_t < - a_lim_t:
				a_t = - a_lim_t
			elif desired_a_t > a_lim_t:
				a_t = a_lim_t
			return curSpeed + a_t * STEPT
		else:
			raise NotImplementedError




