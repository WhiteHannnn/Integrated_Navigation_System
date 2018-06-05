import numpy 

class INS_Data():
	Accel = numpy.zeros((3,1))
	Gyro = numpy.zeros((3,1))

class GPS_Data():
	Pos = numpy.zeros((3,1))
	Vel = numpy.zeros((3,1))

class Wheel_Data():
	Vel = numpy.zeros((3,1))

class Magnet_Data():
	Ang = numpy.zeros((3,1))
	