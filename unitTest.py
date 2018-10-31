# Unit test for track.py


from track import *
import math
import random





def test_steering():
	def createTest(steering, initAng, rotate):
		out = steering.rotateVelocity((math.cos(math.radians(initAng)), math.sin(math.radians(initAng))), -rotate)
		assert(angEqual(out, (math.cos(math.radians(initAng + rotate)), math.sin(math.radians(initAng + rotate)))))
	
	def angEqual(ag1, ag2):
		if abs(ag1[0] - ag2[0]) < 2 and abs(ag1[1] - ag2[1]) < 2:
			return True
		else:
			print("diff:", abs(ag1[0] - ag2[0]), abs(ag1[1] - ag2[1]))
			return False

	maxAng = 60
	steering = Steering(maxAng)	
	createTest(steering, 0, -45)
	createTest(steering, 45, -45)
	createTest(steering, 90, -45)
	createTest(steering, -90, -45)

	createTest(steering, 125, 35)
	createTest(steering, 12.4, 35)

	for i in range(1000):
		ia = random.randint(-360, 360)
		ro = random.randint(-maxAng + 1, maxAng - 1)
		createTest(steering, ia, ro)

	print("testing getAC")
	print(steering.getAC(4, -1, 9.8 * 0.7))
	print(steering.getAC(4, 1, 9.8 * 0.7))
	print(steering.getAC(0.5, -1, 9.8 * 0.7))
	print(steering.getAC(0.5, 1, 9.8 * 0.7))
	print(steering.getAC(0.4, -1, 9.8 * 0.7))
	print(steering.getAC(0.4, 1, 9.8 * 0.7))
	print(steering.getAC(0.3, -1, 9.8 * 0.7))
	print(steering.getAC(0.3, 1, 9.8 * 0.7))




test_steering()


