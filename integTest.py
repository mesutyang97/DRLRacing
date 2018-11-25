# integral testing

from track import *
from controller import *

def setup(numCars):
	dt_lst = [((15, 15), 3), ((15, 45), 3)]
	bd_lst = [((15, 15), (15, 45), 1/4)]
	fl = ((0, 30), (15, 30), 3, (0, 1))
	track = Track(0.8, 0.3, 30, 60, bd_lst, dt_lst, fl)


	# First car
	startLocation = (5, 10)
	startVelocity = (0, 0.00001)
	
	#1.35 kg
	mass = 1.35

	# 0% drag
	drag = 0

	# 2 m/s
	topSpeed = 2

	# 20 degrees
	maxTurningAngle = 20

	# 400 mm
	length = 400

	# 190 mm
	width = 190

	controller_lst = []

	for i in range(numCars):

		carState_i = CarState(startLocation - i * (0, 0.5), startVelocity, i + 1, mass, drag, topSpeed, maxTurningAngle, length, width, True)
		car_i = Car(carState_i, i + 1, length, width)

		track.initializeCar(carState_i)
		controller_i = Controller(track, car_i)
		controller_lst.append(controller_i)

	

	return controller_lst





ctrl_lst = setup()

maxStep = 10

for i in range(maxStep)







setup()
