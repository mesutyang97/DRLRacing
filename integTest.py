# integral testing


from track import *
from controller import *
import numpy as np
import random
import matplotlib.pyplot as plt

def setup(numCars):
	dt_lst = [((10, 10), 3), ((10, 30), 3)]
	bd_lst = [((10, 10), (10, 30), 6)]
	fl = ((0, 20), (7, 20), 3, (0, 1))
	track = Track(0.8, 0.3, 20, 40, bd_lst, dt_lst, fl)


	# First car
	poleLocation = (7, 8)
	startVelocity = (0, 0.00001)
	
	#1.35 kg
	mass = 1.35

	# 0% drag
	drag = 0

	# 2 m/s
	topSpeed = 3

	# 20 degrees
	maxTurningAngle = 20

	# 400 mm
	length = 400

	# 190 mm
	width = 190

	controller_lst = []

	for i in range(numCars):
		startLocation = np.subtract(poleLocation, np.multiply(i, (0, 3)))

		carState_i = CarState(startLocation, startVelocity, i + 1, mass, drag, topSpeed, maxTurningAngle, length, width, True)
		car_i = Car(carState_i, i + 1, length, width)

		track.initializeCar(carState_i)
		controller_i = Controller(track, car_i)
		controller_lst.append(controller_i)

	g = track.getGrid()
	print(g.shape)
	plt.imshow(g, cmap='gray')
	plt.show()

	return track, controller_lst





t, ctrl_lst = setup(1)

maxStep = 100

for i in range(maxStep):
	for ctrl in ctrl_lst:
		rd = random.uniform(-0.8, 0.8)
		ctrl.step((rd, 1.0))

	if i % 10 == 0:
		g = t.getGrid()
		print(g.shape)
		plt.imshow(g, cmap='gray')
		plt.show()


