import numpy
import math
import rospy

class INS():
	# Constant
	pi = 3.1415926
	deg = 180 / pi
	rad = pi / 180
	wie = 4.167 * 10 ** (-3) * pi / 180
	e = 1/297
	Re = 6378137
	g = 9.7803267714

	def __init__(self):
		self.Ang = numpy.zeros((3,1))		# Body to ENU
		self.Vel = numpy.zeros((3,1))		# ENU
		self.Pos = numpy.zeros((3,1))		# longitude, latitude, altitude
		self.Gyro_bias = numpy.zeros((3,1))	# Gyro bias, rad/s
		self.Acc_bias = numpy.zeros((3,1))	# Acce bias, m/s^2
		self.DCM_b2n = numpy.zeros((3,3))	# Direction Cosine Matrix body to navigation axis
		self.Quaternion = numpy.zeros((4,1))# Quaternion of attitude 
		self.wien = numpy.zeros((3,1))
		self.wenn = numpy.zeros((3,1))

	def Initial_alignment(self, gyro, acce):#, mag, pos, vel):
		self.Pos[0] = 116
		self.Pos[1] = 39
		self.Pos[2] = 1000

		self.Vel[0] = 0
		self.Vel[1] = 0
		self.Vel[2] = 0

		self.Ang[0] = 0
		self.Ang[1] = 0
		self.Ang[2] = 0

		self.Gyro_bias[0] = gyro[0]
		self.Gyro_bias[1] = gyro[1]
		self.Gyro_bias[2] = gyro[2]

		self.Acc_bias[0] = acce[0]
		self.Acc_bias[1] = acce[1]
		self.Acc_bias[2] = acce[2]

		self.DCM_b2n[0,0] = 1
		self.DCM_b2n[1,1] = 1
		self.DCM_b2n[2,2] = 1

		self.Quaternion[0] = 1

		flag = True

		return flag

	def Update(self, gyro_origin, acce_origin, error, Delta_T):

		gyro, acce, Error = self.Correct(gyro_origin, acce_origin, error)

		# 
		self.wien[0] = 0
		self.wien[1] = self.wie * math.cos( self.Pos[1] )
		self.wien[2] = self.wie * math.sin( self.Pos[1] )

		#
		self.Rm = self.Re * (1 - 2 * self.e + 3 *self. e * math.sin( self.Pos[1] )**2)
		self.Rn = self.Re * (1 + self.e * math.sin( self.Pos[1] )**2)
		self.wenn[0] = self.Vel[1] / (self.Rm + self.Pos[2])
		self.wenn[1] = self.Vel[0] / (self.Rn + self.Pos[2])
		self.wenn[2] = self.Vel[0] * math.tan( self.Pos[1]) / (self.Rn + self.Pos[2])

		# Attitude rate
		DAng_rate = ( gyro - self.DCM_b2n.T.dot( (self.wien + self.wenn) ) ) * Delta_T

		# Accelerate
		Accen = self.DCM_b2n.dot(acce)

		g_vector = numpy.zeros((3,1))
		g_vector[2] = - self.g
		V_cross_w = numpy.zeros((3,1))
		V_cross_w[0] = (2 * self.wien[1] + self.wenn[1]) * self.Vel[2] - (2 * self.wien[2] + self.wenn[2]) * self.Vel[1]
		V_cross_w[1] = (2 * self.wien[2] + self.wenn[2]) * self.Vel[0] - (2 * self.wien[0] + self.wenn[0]) * self.Vel[2]
		V_cross_w[2] = (2 * self.wien[0] + self.wenn[0]) * self.Vel[1] - (2 * self.wien[1] + self.wenn[1]) * self.Vel[0]
		DV = Accen - g_vector - V_cross_w

		# update attitude
		DAng = math.sqrt( DAng_rate[0] ** 2 + DAng_rate[1] ** 2 + DAng_rate[2] ** 2 )

		Delta_Ang = numpy.zeros((4,4))
		Delta_Ang[0,1] = -DAng_rate[0]
		Delta_Ang[0,2] = -DAng_rate[1]
		Delta_Ang[0,3] = -DAng_rate[2]

		Delta_Ang[1,0] = DAng_rate[0]
		Delta_Ang[1,2] = DAng_rate[2]
		Delta_Ang[1,3] = - DAng_rate[1]

		Delta_Ang[2,0] = DAng_rate[1]
		Delta_Ang[2,1] = - DAng_rate[2]
		Delta_Ang[2,3] = DAng_rate[0]

		Delta_Ang[3,0] = DAng_rate[2]
		Delta_Ang[3,1] = DAng_rate[1]
		Delta_Ang[3,2] = - DAng_rate[0]

		I = numpy.eye((4))
		self.Quaternion = ( ( 1 - DAng ** 2 / 8 ) * I + ( 0.5 - DAng ** 2 ) / 48 * Delta_Ang ).dot (self.Quaternion )
		D = math.sqrt( self.Quaternion[0] ** 2 + self.Quaternion[1] ** 2 + self.Quaternion[2] ** 2 + self.Quaternion[3] ** 2 )
		self.Quaternion = self.Quaternion / D
		a0 = self.Quaternion[0]
		a1 = self.Quaternion[1]
		a2 = self.Quaternion[2]
		a3 = self.Quaternion[3]

		self.DCM_b2n[0,0] = a0 ** 2 + a1 ** 2 - a2 ** 2 + a3 ** 2
		self.DCM_b2n[0,1] = 2 * ( a1 * a2 - a0 * a3)
		self.DCM_b2n[0,2] = 2 * ( a1 * a3 + a0 * a2)

		self.DCM_b2n[1,0] = 2 * ( a1 * a2 + a0 * a3 )
		self.DCM_b2n[1,1] = a0 ** 2 - a1 ** 2 + a2 ** 2 - a3 ** 2
		self.DCM_b2n[1,2] = 2 * ( a2 * a3 - a0 * a1 )

		self.DCM_b2n[2,0] = 2 * ( a1 * a3 - a0 * a2 )
		self.DCM_b2n[2,1] = 2 * ( a2 * a3 + a0 * a1 )
		self.DCM_b2n[2,2] = a0 ** 2 - a1 ** 2 - a2 ** 2 + a3 ** 2

		# Pitch
		self.Ang[0] = math.asin( self.DCM_b2n[2,1] )

		# Roll
		if self.DCM_b2n[2,2] != 0:
			roll_main = math.atan( -self.DCM_b2n[2,0] / self.DCM_b2n[2,2] )
		else:
			self.Ang[1] = pi / 2

		if -self.DCM_b2n[2,0] > 0:                                               
			if self.DCM_b2n[2,2] > 0: 
				self.Ang[1] = roll_main                             
			else:                                                                   
				self.Ang[1] = roll_main + INS.pi                             
		elif -self.DCM_b2n[2,0] == 0: 
			self.Ang[1] = 0
		else:                                                                    
			if self.DCM_b2n[2,2] > 0:                                                 
				self.Ang[1] = roll_main                                  
			else:                                                                
				self.Ang[1] = roll_main - INS.pi                           

		# Yaw
		if self.DCM_b2n[1,1]  != 0:
			yaw_main = math.atan( self.DCM_b2n[0,1] / self.DCM_b2n[1,1] )
		else:
			self.Ang[2] = pi / 2

		if self.DCM_b2n[0,1] > 0:
			if self.DCM_b2n[1,1] > 0:
				self.Ang[2] = yaw_main
			else:
				self.Ang[2] = yaw_main + INS.pi
		elif self.DCM_b2n[0,1] == 0:
 			self.Ang[2] = 0
 		else:
 			if self.DCM_b2n[1,1] > 0:
				self.Ang[2] = yaw_main
			else:
				self.Ang[2] = yaw_main - INS.pi

		# Update velocity
		Vel_pre = self.Vel
		self.Vel = self.Vel + DV * Delta_T

		# Update position
		# type 1
		self.Pos[0] = self.Pos[0] + self.Vel[0] * Delta_T / ( ( self.Rn + self.Vel[2] ) * math.cos( self.Pos[1] ) )
		self.Pos[1] = self.Pos[1] + self.Vel[1] * Delta_T / ( self.Rm + self.Vel[2] )
		self.Pos[2] = self.Pos[2] + self.Vel[2] * Delta_T

		# type 2
		self.Pos[0] = self.Pos[0] + 0.5 * ( self.Vel[0] + Vel_pre[0] ) * Delta_T / ( ( self.Rn + self.Vel[2] ) * math.cos( self.Pos[1] ) )
		self.Pos[1] = self.Pos[1] + 0.5 * ( self.Vel[1] + Vel_pre[1] ) * Delta_T / ( self.Rm + self.Vel[2] )
		self.Pos[2] = self.Pos[2] + 0.5 * ( self.Vel[2] + Vel_pre[2] ) * Delta_T

		return [self.Pos, self.Vel, self.Ang, self.DCM_b2n, Error]

	def Correct(self, gyro, acc, Error):
		self.Pos[0] = self.Pos[0] - Error[1]
		self.Pos[1] = self.Pos[1] - Error[0]
		self.Pos[2] = self.Pos[2] - Error[2]

		self.Vel = self.Vel - Error[3:6]

		self.Ang = self.Ang - Error[6:9]

		gyro = gyro - Error[9:12]

		acc = acc - Error[12:15]

		self.Quaternion[0] = math.cos(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) + \
						math.sin(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)
		self.Quaternion[1] = math.sin(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) + \
						math.cos(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)
		self.Quaternion[2] = math.cos(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) - \
						math.sin(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)
		self.Quaternion[3] = math.sin(self.Ang[0] / 2) * math.sin(self.Ang[1] / 2) * math.cos(self.Ang[2] / 2) - \
						math.cos(self.Ang[0] / 2) * math.cos(self.Ang[1] / 2) * math.sin(self.Ang[2] / 2)

		error = numpy.zeros( (15, 1) )

		#self.DCM_b2n[0,0] = math.cos(self.Ang[1]) * math.cos(self.Ang[2]) + \
		#				math.sin(self.Ang[0]) * math.sin(self.Ang[1]) * math.sin(self.Ang[2])
		#self.DCM_b2n[0,1] = - math.cos(self.Ang[1]) * math.sin(self.Ang[2]) + \
		#				math.sin(self.Ang[0]) * math.sin(self.Ang[1]) * math.cos(self.Ang[2])
		#self.DCM_b2n[0,2] = - math.sin(self.Ang[1]) * math.cos(self.Ang[0])

		#self.DCM_b2n[1,0] = math.cos(self.Ang[0]) * math.sin(self.Ang[2])
		#self.DCM_b2n[1,1] = math.cos(self.Ang[0]) * math.cos(self.Ang[2])
		#self.DCM_b2n[1,2] = math.sin(self.Ang[0])

		#self.DCM_b2n[2,0] = math.sin(self.Ang[1]) * math.cos(self.Ang[2]) + \
		#				math.sin(self.Ang[0]) * math.cos(self.Ang[1]) * math.sin(self.Ang[2])
		#self.DCM_b2n[2,1] = - math.sin(self.Ang[1]) * math.sin(self.Ang[2]) - \
		#				math.sin(self.Ang[0]) * math.cos(self.Ang[1]) * math.cos(self.Ang[2])
		#self.DCM_b2n[2,2] = math.cos(self.Ang[0]) * math.cos(self.Ang[1])

		return gyro, acc, error
