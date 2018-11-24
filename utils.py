# utils.py

import math

def rotateVector(v, dtheta):
	old_theta = 0
	if v[0] == 0:
		if v[1] > 0:
			print("used 1")
			old_theta = 1.5707963267948966
		else:
			print("used 2")
			old_theta = - 1.5707963267948966
	else:
		old_theta = math.tan(v[1]/v[0])

	new_theta = old_theta + math.radians(dtheta)
	return (math.cos(new_theta), math.sin(new_theta))


def ftToMm(lengthInFeet):
	return int(lengthInFeet * 304.8)