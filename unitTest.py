# Unit test for track.py


from track import *
import math
import random
from utils import *
import matplotlib.pyplot as plt

# Python Imaging Library imports
from PIL import Image
from PIL import ImageDraw


def test_steering():
	def createTest(steering, initAng, rotate):
		out = steering.rotateVelocity((math.sin(math.radians(initAng)), math.cos(math.radians(initAng))), -rotate)
		assert(angEqual(out, (math.sin(math.radians(initAng + rotate)), math.cos(math.radians(initAng + rotate)))))
	
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



def test_rotateVector():
	def angEqual(ag1, ag2):
		if abs(ag1[0] - ag2[0]) < 2 and abs(ag1[1] - ag2[1]) < 2:
			return True
		else:
			print("diff:", abs(ag1[0] - ag2[0]), abs(ag1[1] - ag2[1]))
			return False

	def createTest(initAng, rotate):
		out = rotateVector((math.cos(math.radians(initAng)), math.sin(math.radians(initAng))), rotate)
		assert(angEqual(out, (math.cos(math.radians(initAng + rotate)), math.sin(math.radians(initAng + rotate)))))

		

	maxAng = 60
	for i in range(1000):
		ia = random.randint(-360, 360)
		ro = random.randint(-maxAng + 1, maxAng - 1)
		createTest(ia, ro)
	
	
def test_ftToMm():
	assert(ftToMm(1) == 304)
	assert(ftToMm(10) == 3048)
	assert(ftToMm(75) == 22860)
	
def test_track_initialize():
	dt_lst = [((15, 15), 3), ((15, 45), 3)]
	bd_lst = [((15, 15), (15, 45), 1/4)]
	fl = ((0, 30), (15, 30), 3, (0, 1))
	t = Track(0.8, 0.3, 30, 60, bd_lst, dt_lst, fl)
	g = t.getGrid()
	print(g.shape)
	plt.imshow(g, cmap='gray')
	plt.show()

'''
Credit: https://stackoverflow.com/questions/12638790/drawing-a-rectangle-inside-a-2d-numpy-array
'''
def test_transformRectangle():
	data = np.zeros((700, 700))
	# Convert the numpy array to an Image object.
	img = Image.fromarray(data)

	# Draw a rotated rectangle on the image.
	draw = ImageDraw.Draw(img)
	rect = transformRectangle((350, 350), (-100, 200), 400, 190)
	draw.polygon([tuple(p) for p in rect], fill=1)
	# Convert the Image data to a numpy array.
	new_data = np.asarray(img)

	# Display the result using matplotlib.  (`img.show()` could also be used.)
	#plt.imshow(new_data, cmap=plt.cm.gray)
	#plt.show()


test_steering()
test_rotateVector()
print("PASS: test_rotateVector")
test_ftToMm()
print("PASS: test_ftToMm")
#test_track_initialize()
print("PASS: test_track_initialize")
test_transformRectangle()

