import numpy as np
import math

# Position: not location, but where you
# tInput: throttle input
# sInput: steering input: -1: left. +1: right

STEPT = 0.1

class Track:
	def __init__(self, miu):
		# Friction
		self.miu = miu



class Car:
	def __init__(self, startingState, carNumber, startingRanking, mass, drag):
		self.carNumber = carNumber
		self._mass = mass
		self.a = a



class CarState:
	def __init__(self, startLocation, startVelocity, startRanking, mass, drag, topSpeed, maxTurningAngle, isLinear = True):
		self._location = startLocation
		self._velocity = startVelocity
		self._rank = startRanking
		self._mass = mass
		self._drag = drag
		self._throttle = ()
		self._steering = Steering(maxTurningAngle)
		self._throttle = Throttle(drag, topSpeed, isLinear)

	def getLocation(self):
		return self._location

	def getVelocity(self):
		return self._velocity

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
		self._velocity = nextSpeed * nextVelocity_unit
		self._location = self._location + self._velocity * STEPT




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










