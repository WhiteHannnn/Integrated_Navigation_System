import numpy
import math
import rospy

from INS import *

class KF():

	pi = 3.1415926
	deg = 180 / pi
	rad = pi / 180
	wie = 4.167 * 10 ** (-3) * pi / 180
	e = 1/297
	Re = 6378137
	g = 9.7803267714

	# default
	default_dim_x = 15	# Pos, Vel, Ang, gyro, acce 
	default_dim_z = 10	# Pos, Vel, Ang, Wheel_Speed

	def __init__(self):
		self.dim_x = KF.default_dim_x
		self.dim_z = KF.default_dim_z

		self.A = numpy.zeros( (self.dim_x, self.dim_x) )
		self.H = numpy.zeros( (self.dim_z, self.dim_x) )

		self.Q = numpy.zeros( (self.dim_x, self.dim_x) )
		self.R = numpy.zeros( (self.dim_z, self.dim_z) )

		self.X_prediction = numpy.zeros( (self.dim_x, 1) )
		self.X = numpy.zeros( (self.dim_x, 1) )
		self.P_prediction = numpy.zeros( (self.dim_x, self.dim_x) )
		self.P = numpy.zeros( (self.dim_x, self.dim_x) )
		self.K = numpy.zeros( (self.dim_x, self.dim_z) )

	def Kalman_Filter(self, Pos, Vel, Ang, Cnb, Gyro, Acc, Delta_T):

		self.Construct_A(Pos, Vel, Cnb, Acc)
		self.Construct_H()
		self.Construct_Q()
		self.Construct_R()

		Z = numpy.zeros( (10, 1) )
		Z[0] = Pos[1] - 39 
		Z[1] = Pos[0] - 116
		Z[2] = Pos[2] - 1000
		Z[3] = Vel[0] - 0
		Z[4] = Vel[1] - 0 
		Z[5] = Vel[2] - 0
		Z[6] = Ang[0] - 0
		Z[7] = Ang[1] - 0
		Z[8] = Ang[2] - 0 
		Z[9] = 0 

		F = numpy.zeros( (15, 15) )
		F = numpy.eye(15) + self.A * 0.1 / Delta_T

		self.X_prediction = F.dot(self.X)
		self.P_prediction = F.dot(self.P.dot(F.T)) + self.Q
		self.K = self.P_prediction.dot(self.H.T) \
				.dot( numpy.linalg.inv(self.H.dot(self.P_prediction).dot(self.H.T) + self.R) ) 
		self.X = self.X_prediction + self.K.dot( Z - self.H.dot(self.X_prediction) )
		self.P = ( numpy.eye(15) - self.K.dot(self.H) ).dot(self.P_prediction).dot( (numpy.eye(15) - self.K.dot(self.H)).T )\
					+ self.K.dot(self.R).dot(self.K.T)

		return self.X

	def Construct_A(self, Pos, Vel, Cnb, Acc):
		Rm = self.Re * (1 - 2 * self.e + 3 *self. e * math.sin( Pos[1] ) ** 2)
		Rn = self.Re * (1 + self.e * math.sin( Pos[1] ) ** 2)

		self.A[0, 2] = - Vel[1] / (Rm + Pos[2]) ** 2
		self.A[0, 4] = 1 / (Rm + Pos[2])
		self.A[1, 0] = Vel[0] / (Rn + Pos[2]) / math.cos(Pos[1]) * math.tan(Pos[1])
		self.A[1, 2] = - Vel[0] / (Rn + Pos[2]) ** 2 / math.cos(Pos[1])
		self.A[1, 3] = 1 / math.cos(Pos[1]) / (Rn + Pos[2])
		self.A[2, 5] = 1;
		self.A[3, 0] = 2 * KF.wie * Vel[1] * math.cos(Pos[1]) + \
						Vel[0] * Vel[1] / (Rn + Pos[2]) / math.cos(Pos[1]) ** 2 + \
					    2 * KF.wie * Vel[2] * math.sin(Pos[1])
		self.A[3, 2] = Vel[0] * Vel[2] / (Rn + Pos[2]) ** 2 - \
						Vel[0] * Vel[1] * math.tan(Pos[1])
		self.A[3, 3] = Vel[1] * math.tan(Pos[1]) / (Rm + Pos[2]) - Vel[2] / (Rm + Pos[2])
		self.A[3, 4] = 2 * KF.wie * math.sin(Pos[1]) + Vel[0] * math.tan(Pos[1]) / (Rn + Pos[2])
		self.A[3, 5] = - ( 2 * KF.wie * math.cos(Pos[1]) + Vel[0] / (Rn + Pos[2]) )
		self.A[3, 7] = - Acc[2]
		self.A[3, 8] = Acc[1]
		self.A[3, 12] = Cnb[0, 0]
		self.A[3, 13] = Cnb[0, 1]
		self.A[3, 14] = Cnb[0, 2]
		self.A[4, 0] = - ( 2 * KF.wie * Vel[0] * math.cos(Pos[1]) + \
							Vel[0] ** 2 / ( math.cos(Pos[1]) ) ** 2 / (Rn + Pos[2]) )
		self.A[4, 2] = ( Vel[0] ** 2 * math.tan(Pos[1]) + Vel[1] * Vel[2] ) / (Rn + Pos[2]) ** 2
		self.A[4, 3] = - 2 * ( KF.wie * math.sin(Pos[1]) + Vel[0] * math.tan(Pos[1]) / (Rn + Pos[2]) )
		self.A[4, 4] = - Vel[2] / (Rm + Pos[2])
		self.A[4, 5] = - Vel[1] / (Rm + Pos[2])
		self.A[4, 6] = Acc[2]
		self.A[4, 8] = - Acc[0]
		self.A[4,12] = Cnb[1, 0]
		self.A[4,13] = Cnb[1, 1]
		self.A[4,14] = Cnb[1, 2]
		self.A[5, 0] = - 2 * KF.wie * Vel[0] * math.sin(Pos[1])
		self.A[5, 2] = - ( Vel[0] ** 2 + Vel[1] ** 2 ) / (Rn + Pos[2]) ** 2
		self.A[5, 3] = 2 * ( KF.wie * math.cos(Pos[1]) + Vel[0] / (Rn + Pos[2]) )
		self.A[5, 4] = 2 * Vel[1] / (Rm + Pos[2])
		self.A[5, 6] = - Acc[1]
		self.A[5, 7] =  Acc[0]
		self.A[5, 12] = Cnb[2, 0]
		self.A[5, 13] = Cnb[2, 1]
		self.A[5, 14] = Cnb[2, 2]
		self.A[6, 2] = Vel[1] / (Rm + Pos[2]) ** 2
		self.A[6, 4] = - 1 / (Rm + Pos[2])
		self.A[6, 7] = KF.wie * math.sin(Pos[1]) + Vel[0] * math.tan(Pos[1]) / (Rn + Pos[2])
		self.A[6, 8] = - ( KF.wie * math.cos(Pos[1]) + Vel[0] / (Rn + Pos[2]) )
		self.A[6, 9] = Cnb[0, 0]
		self.A[6, 10] = Cnb[0, 1]
		self.A[6, 11] = Cnb[0, 2]
		self.A[7, 2] = - Vel[0] / (Rn + Pos[2])
		self.A[7, 3] = 1 / (Rn + Pos[2])
		self.A[7, 6] = - ( KF.wie * math.sin(Pos[1]) + Vel[0] * math.tan(Pos[1]) / (Rn + Pos[2]) )
		self.A[7, 8] = - Vel[1] / (Rm + Pos[2])
		self.A[7, 9] = Cnb[1, 0]
		self.A[7, 10] = Cnb[1, 1]
		self.A[7, 11] = Cnb[1, 2]
		self.A[8, 0] = KF.wie * math.cos(Pos[1]) + Vel[0] / math.cos(Pos[1]) ** 2 / (Rn + Pos[2])
		self.A[8, 2] = - KF.wie * math.tan(Pos[1]) / (Rn + Pos[2]) ** 2
		self.A[8, 3] = math.tan(Pos[1]) / (Rn + Pos[2])
		self.A[8, 6] = KF.wie * math.cos(Pos[1]) + Vel[0] / (Rn + Pos[2])
		self.A[8, 7] = Vel[1] / (Rm + Pos[2])
		self.A[8, 9] = Cnb[2, 0]
		self.A[8, 10] = Cnb[2, 1]
		self.A[8, 11] = Cnb[2, 2]
		self.A[9, 9] = - 1 / 300
		self.A[10, 10] = - 1 / 300
		self.A[11, 11] = - 1 / 300
		self.A[12, 12] = - 1 / 300
		self.A[13, 13] = - 1 / 300
		self.A[14, 14] = - 1 / 300


	def Construct_H(self):
		self.H[0:10, 0:10] = numpy.eye(10)

	def Construct_Q(self):
		self.Q[0:3, 0:3] = numpy.eye(3)
		self.Q[3:6, 3:6] = numpy.eye(3)
		self.Q[6:9, 6:9] = numpy.eye(3)
		self.Q[9:12, 9:12] = numpy.eye(3)
		self.Q[12:15, 12:15] = numpy.eye(3)
	
	def Construct_R(self):
		self.R[0:3, 0:3] = numpy.eye(3)	* 0.01	# Ang receive error
		self.R[3:6, 3:6] = numpy.eye(3) * 0.01	# 
		self.R[6:9, 6:9] = numpy.eye(3) * 0.01 #
		self.R[9, 9] = 0.01




