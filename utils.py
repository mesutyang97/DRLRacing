# utils.py

import math
import numpy as np

def rotateVector(v, dtheta):
	old_theta = 0
	if v[1] == 0:
		# ugly fix
		denom = 0.001
	else:
		denom = v[1]
		
	old_theta = np.arctan(v[0]/denom)

	new_theta = old_theta + math.radians(dtheta)
	return (math.cos(new_theta), math.sin(new_theta))


def ftToMm(lengthInFeet):
	return int(lengthInFeet * 304.8)

def sameDirection(v1, v2):
	return np.dot(np.array(v1), v2) > 0


def transformRectangle(loc, v, length, width):

	if v[1] == 0:
		denom = 0.001
	else:
		denom = v[1]
	theta = np.arctan(v[0]/denom)
	rect = np.array([(0, 0), (length, 0), (length, width), (0, width), (0, 0)])
	R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
	x, y = np.array(loc) - np.dot(np.array([length/2, width/2]), R) 
	offset = np.array([x, y])
	transformed_rect = np.dot(rect, R) + offset
	return transformed_rect