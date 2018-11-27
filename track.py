# track.py
# This file contains all the necessary component to simulate a ractrack

import numpy as np
import math
import utils
import cv2

# Python Imaging Library imports
from PIL import Image
from PIL import ImageDraw

import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Position: not location, but where you
# tInput: throttle input
# sInput: steering input: -1: left. +1: right


RECORD = False

STEPT = 0.1

COLLISONPEN = 3.0

DOWNSAMPLED = True

# Here are some default parameters for a oval track
w_ft = 20
l_ft = 40
bd_lst = [((10, 10), (10, 30), 6), ((7, 0.8), (0.8, 7), 2.5), ((7, 39.2), (0.8, 33), 2.5), ((13, 39.2), (19.8, 33), 2.5), ((13, 0.8), (19.8, 7), 2.5)]
dt_lst = [((10, 10), 3), ((10, 30), 3)]
boardwith_ft = 1.63
fl = ((boardwith_ft, 20), (7, 20), 3, (0, 1))



poleLocation = (7, 8)
lineupSpace = (0, 3)
sVelocity = (0, 0.00001)

class Track:
	def __init__(self, miu, dot_miu, w_feet = w_ft, l_feet = l_ft, board_lst = bd_lst, dot_lst = dt_lst, finish_line = fl):
		# Friction

		self.miu = miu
		self._trackw = utils.ftToMm(w_feet)
		self._trackl = utils.ftToMm(l_feet)
		boardwidth = utils.ftToMm(boardwith_ft)

		self._grid = self.initializeGrid(self._trackw, self._trackl, boardwidth)
		self._dot_lst = dot_lst
		self._finish_line = finish_line
		self._board_lst = board_lst
		self.buildDots(dot_lst, self._trackw, self._trackl)
		self.buildFinishLine(finish_line)
		self.buildBoards(board_lst)
		self.carDict = {}


	'''
	Initialize a 1-padded zero arrays as the grid
	'''
	def initializeGrid(self, w, l, boardwidth):
		g = np.zeros((w, l))
		# Zero pad the boarders
		g[0:boardwidth,:] = np.ones((boardwidth, l))
		g[w-boardwidth:w,:] = np.ones((boardwidth, l))
		g[:,0:boardwidth] = np.ones((w, boardwidth))
		g[:,l-boardwidth:l] = np.ones((w, boardwidth))
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
			'''
			assert endpoint_A_y <= endpoint_B_y, "Endpoint invariant in y violated"
			assert endpoint_A_x <= endpoint_B_x, "Endpoint invariant in x violated"
			'''

			half_width = utils.ftToMm(board[2]/2)
			# Board is horizontal, or, aligned with length
			if endpoint_A_y == endpoint_B_y:
				self._grid[endpoint_A_y - half_width:endpoint_A_y + half_width, endpoint_A_x: endpoint_B_x] = np.ones(((2*half_width), endpoint_B_x- endpoint_A_x))
			# Board is vertical, or, aligned with width
			elif endpoint_A_x == endpoint_B_x:
				self._grid[endpoint_A_y: endpoint_B_y, endpoint_A_x - half_width:endpoint_A_x + half_width] = np.ones((endpoint_B_y- endpoint_A_y, (2*half_width)))
			# Supporting diagonal board
			else:
				midpoint_y = int((endpoint_A_y + endpoint_B_y)/2)
				midpoint_x = int((endpoint_A_x + endpoint_B_x)/2)
				v = np.subtract((endpoint_B_y, endpoint_B_x), (endpoint_A_y, endpoint_A_x))
				length = np.linalg.norm(v)
				width = half_width * 2
				board = utils.transformRectangle((midpoint_y, midpoint_x), v, length, width)
				self.updateGrid(board, 1)

	'''
	The dot_lst will be a list of form
	((y,x), radius), in feets
	'''
	def buildDots(self, dot_lst, w, l):
		temp_g = self._grid.copy()
		for dot in dot_lst:
			y_ft, x_ft = dot[0]
			y = utils.ftToMm(y_ft)
			x = utils.ftToMm(x_ft)

			r_ft = dot[1]
			r = utils.ftToMm(r_ft)

			a,b = np.ogrid[-y:w-y, -x:l-x]
			mask = a*a + b*b <= r*r

			temp_g[mask] = 0.5
		self._grid = temp_g
	

	'''
	The finish line will be in the form of
	((endpoint_A_y, endpoint_A_x), (endpoint_B_y, endpoint_B_x), width, (finish_line_dir_vec))
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
			self._grid[endpoint_A_y - half_width:endpoint_A_y + half_width, endpoint_A_x: endpoint_B_x] \
			= -0.1 * np.ones(((2*half_width), endpoint_B_x- endpoint_A_x))
		# Board is vertical, or, aligned with width
		elif endpoint_A_x == endpoint_B_x:
			self._grid[endpoint_A_y: endpoint_B_y, endpoint_A_x - half_width:endpoint_A_x + half_width] \
			= -0.1 * np.ones((endpoint_B_y- endpoint_A_y, (2*half_width)))
		# No support of diagonal board yet
		else:
			print("Error: do not support diagonal board")

		#Finish line direction
		self.finish_line_dir = np.array(finish_line[3])


	def rebuildTrack(self):
		self.buildDots(self._dot_lst, self._trackw, self._trackl)
		self.buildFinishLine(self._finish_line)
		self.buildBoards(self._board_lst)


	# Initialize the grid in the beginning
	# carInitS: the initial state of the car initialized
	def initializeCar(self, carInitS):
		init_rect = carInitS.getTransformedRectangle()
		self.updateGrid(init_rect, 1)
		return None


	def updateGrid(self, transformed_rect, filling):
		img = Image.fromarray(self._grid)
		draw = ImageDraw.Draw(img)
		draw.polygon([tuple(p) for p in transformed_rect], fill=filling)
		self._grid = np.asarray(img)


	def getGrid(self):
		return self._grid




class Car:
	def __init__(self, startingState, carNumber, length = 400, width = 190):
		self._state = startingState
		self.carNumber = carNumber
		self.length = length
		self.width = width
	def getCarState(self):
		return self._state



class CarState:
	def __init__(self, startRanking = 1, startVelocity = sVelocity, mass=1.35, 
		drag=0, topSpeed = 6, maxTurningAngle = 30, length = 400, width = 190, env_window_w = 1000, 
		obs_window_w = 10, sensor_only = 1, isLinear = True, max_total_T = 200):
		startLocation_ft = np.subtract(poleLocation, np.multiply(startRanking - 1, lineupSpace))
		startLocation_y = utils.ftToMm(startLocation_ft[0])
		startLocation_x = utils.ftToMm(startLocation_ft[1])
		self._location = np.array([startLocation_y, startLocation_x])
		self._velocity = np.array(startVelocity)
		self._rank = startRanking
		self._mass = mass
		self._drag = drag
		self._steering = Steering(maxTurningAngle)
		self._throttle = Throttle(drag, topSpeed, isLinear)
		self._topSpeed = topSpeed
		self._length = length
		self._width = width
		self._env_window_w = env_window_w
		self._obs_window_w = obs_window_w
		self._sensor_only = sensor_only
		self._recover = False

		self._startCrossing = False
		# own clock
		self._clock = 0

		self._total_T = 0
		self._max_total_T = max_total_T


		self._collision_buff = np.zeros(10)

	def getLocation(self):
		return self._location

	def getVelocity(self):
		return self._velocity

	def getLength(self):
		return self._length

	def getWidth(self):
		return self._width

	def checkCollison(self, nextSpeed, nextVelocity_unit, desired_nextVelocity, desired_nextLocation, curTrack):
		# Move collision buffer by 1
		self._collision_buff[0:9] = self._collision_buff[1:10]
		headPosition = np.multiply(nextVelocity_unit,  (self._length/2)) + desired_nextLocation
		y = int(headPosition[0])
		x = int(headPosition[1])

		# print("next Speed", nextSpeed)
		if not self._recover and curTrack.getGrid()[y][x] == 1 and nextSpeed > 0.9:
			self._recover = True
			self._collision_buff[9] = 1
			# print("collision buffer: ", self._collision_buff)
			return np.multiply(0.0001, nextVelocity_unit), self._location - np.multiply(COLLISONPEN, desired_nextVelocity)
		else:
			self._recover = False
			self._collision_buff[9] = 0
			return desired_nextVelocity, desired_nextLocation

	def getReward(self, curLocation, nextLocation, nextVelocity, curTrack):
		self._clock += 1;
		if np.linalg.norm(nextVelocity) == 0.0001:
			# print("---collision at", curLocation)
			rew = -5
		else:
			rew = -1.5

		# Attemp: speed reward:

		rew = rew + np.abs(np.linalg.norm(nextVelocity) / self._topSpeed)

		y_c = int(curLocation[0])
		x_c = int(curLocation[1])
		y_n = int(nextLocation[0])
		x_n = int(nextLocation[1])

		if (self._startCrossing == False) and utils.sameDirection(nextVelocity, curTrack.finish_line_dir) \
			and curTrack.getGrid()[y_n][x_n] == -0.1:
			self._startCrossing = True
			print("start crossing")
			rew += 1
		
		elif (self._startCrossing == True) and utils.sameDirection(nextVelocity, curTrack.finish_line_dir) \
		and curTrack.getGrid()[y_n][x_n] == 0:
			self._startCrossing = False
			print("finish crossing")
			rew += 1000 * 1/self._clock
			self._clock = 0
		# print("reward: ", rew)
		return rew

	'''
	This is for the filling the grid in the game
	'''
	def getTransformedRectangle(self):
		return utils.transformRectangle(self._location, self._velocity, self._length, self._width)



	def getObservation(self, curTrack):
		im_dst= self.getObservationWindow(curTrack)
		sensor_flatten = np.array(im_dst).flatten()
		if self._sensor_only:
			return sensor_flatten
		else:
			physical_flatten = np.array([self._location, self._velocity]).flatten()
			ob = np.concatenate((sensor_flatten, physical_flatten), axis = 0)
			return ob



	def getObservationWindow(self, curTrack):
		# Get the observation
		# Direction of the car
		v_unit = self._velocity/np.linalg.norm(self._velocity)
		#print("vu", v_unit)
		rotated_v_unit = utils.rotateVector(v_unit, 90)
		#print("rotatedvu", rotated_v_unit)
		#print("self location", self._location)
		new_head = self._location + np.multiply(v_unit, self._length/2)
		#print("new_head", new_head)
		#print("self._env_window_w", self._env_window_w)
		corner = new_head - np.multiply(rotated_v_unit, self._env_window_w/2)
		corner_1 = corner + np.multiply(v_unit, self._env_window_w)
		corner_2 = corner_1 + np.multiply(rotated_v_unit, self._env_window_w)
		corner_3 = corner_2 - np.multiply(v_unit, self._env_window_w)

		pts_src = np.zeros((4, 2))

		# Courtesy to https://www.learnopencv.com/homography-examples-using-opencv-python-c/
		corners = np.array([corner, corner_1, corner_2, corner_3])
		#print("corners")
		#print(corners)
		pts_src[:,0] = corners[:,1]
		pts_src[:,1] = corners[:,0]
		#print("pts_src")
		#print(pts_src)

		pts_dst = np.array([[0, 0],[self._obs_window_w, 0], [self._obs_window_w, self._obs_window_w], [0, self._obs_window_w]])

		# Calculate Homography
		h, status = cv2.findHomography(pts_src, pts_dst)

		im_dst = cv2.warpPerspective(curTrack.getGrid(), h, (self._obs_window_w, self._obs_window_w))
		# print(im_dst)
		return im_dst


	def step(self, sInput, tInput, curTrack, index = 0, enablePrint = False):
		# The current track contains information about other car on the track
		
		if enablePrint:
			print("===========")
			print("Steering Input", sInput)
			print("Throttle Input", tInput)
			print("Current location: ", self._location)
			print("Current velocity: ", self._velocity)
			print("Current speed: ", np.linalg.norm(self._velocity))
		'''
		print("Throttle Input", tInput)
		
		'''
		
		curSpeed = np.linalg.norm(self._velocity)
		curVelocity_unit = self._velocity/curSpeed
		# Assertion
		assert np.linalg.norm(curVelocity_unit) > 0.99 and np.linalg.norm(curVelocity_unit) < 1.01

		a_lim = curTrack.miu * 9.8
		turningAngle, a_c = self._steering.getAC(curSpeed, sInput, a_lim)

		# This step only changed orientation, not magnitude
		nextVelocity_unit = self._steering.rotateVelocity(curVelocity_unit, turningAngle)
		a_lim_t = np.sqrt(np.square(a_lim) - np.square(a_c))
		nextSpeed = self._throttle.getNewSpeed(curSpeed, tInput, a_lim_t)
		desired_nextVelocity = np.multiply(nextSpeed, nextVelocity_unit)
		# The 1000 is to convert m/s to mm/s
		desired_nextLocation = self._location + np.multiply(STEPT * 1000, self._velocity)

		# 1: check collision
		nextVelocity, nextLocation = self.checkCollison(nextSpeed, nextVelocity_unit, desired_nextVelocity, desired_nextLocation, curTrack)
		# 1. check reward for crossing the finish line
		rew = self.getReward(self._location, nextLocation, nextVelocity, curTrack)

		# remove the car from old position
		old_rect = self.getTransformedRectangle()
		curTrack.updateGrid(old_rect, 0)

		self._velocity = nextVelocity
		self._location = nextLocation

		# ========================

		# insert car to new position
		new_rect = self.getTransformedRectangle()
		curTrack.updateGrid(new_rect, 1)

		# Replicated from getObservation
		v_unit = self._velocity/np.linalg.norm(self._velocity)
		rotated_v_unit = utils.rotateVector(v_unit, 90)
		new_head = self._location + np.multiply(v_unit, self._length/2)
		corner = new_head - np.multiply(rotated_v_unit, self._env_window_w/2)

		if RECORD:
			im_dst = self.getObservationWindow(curTrack)
			obsname = "obs_images/" + str(index) + ".png"
			plt.imsave(obsname, im_dst, cmap = 'gray')
			plt.close()

			corner_reorder = (corner[1], corner[0])

			theta = utils.getAngle(v_unit) * 180/math.pi
			#print("theta", theta)

			fig,ax = plt.subplots(1)
			g = curTrack.getGrid()
			ax.imshow(g, cmap='gray')

			rect = patches.Rectangle(corner_reorder,self._env_window_w,self._env_window_w,angle = theta, linewidth=1,edgecolor='r',facecolor='none')
			ax.add_patch(rect)

			gridname = "grid_images/" + str(index) + ".png"
			plt.savefig(gridname, cmap = 'gray')
			plt.close()

		# Put observation together
		ob = self.getObservation(curTrack)

		self._total_T += 1
		# print("sum of cb", np.sum(self._collision_buff))
		done = (self._total_T > self._max_total_T) or np.sum(self._collision_buff) > 3
		# print("remote side done: ", done)
		return ob, rew, done, dict()




class Steering:
	def __init__(self, maxTurningAngle):
		self._maxAngle = maxTurningAngle

	def getAC(self, curSpeed, sInput, a_lim):
		assert(sInput <= 1.0 and sInput >= -1.0)
		# FIXME: hack to make the thing goes right
		sInput = - sInput
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




