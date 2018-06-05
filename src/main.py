#!/usr/bin/env python

import rospy	#python for ros
from parameter import *		# store sensor's data
from INS import *
from KF import *

from sensor_msgs.msg import Imu, MagneticField	 # msg for imu,mag
from tbw_shanqi_msgs.msg import ExtraWheelSpeedReportStamped 	# msg for wheelspeed

INS_Cal = INS()
KF_Cal = KF()

def Callback_Imu(data):		# ENU
	INS_Data.Gyro[0] = data.angular_velocity.x
	INS_Data.Gyro[1] = data.angular_velocity.y
	INS_Data.Gyro[2] = data.angular_velocity.z

	INS_Data.Accel[0] = data.linear_acceleration.x
	INS_Data.Accel[1] = data.linear_acceleration.y
	INS_Data.Accel[2] = - data.linear_acceleration.z

	#rospy.loginfo("Accelerator:%f", INS_Data.Accel[0])

def Callback_Mag(data):		# ENU
	0
	#rospy.loginfo("MagTime:%s", data.header.stamp.nsecs)

def Callback_ExtraWheelSpeedReportStamped(data):
	0
	#rospy.loginfo("Wheelspeed:%s", data.data.front_axle_speed)

def main():
	rospy.init_node('Subscriber_rawimu', anonymous = True)
	rospy.Subscriber('/imu/data', Imu, Callback_Imu)
	rospy.Subscriber('/imu/mag', MagneticField, Callback_Mag)
	rospy.Subscriber('/vehicle/Bbee/extra_wheelspeed_report', ExtraWheelSpeedReportStamped, Callback_ExtraWheelSpeedReportStamped)
	
	rate = rospy.Rate(50)	# set update frequency
	Delta_T = 1 / 50
	
	# initial alignment
	Flag = False
	while not Flag:
		Flag = INS_Cal.Initial_alignment(INS_Data.Gyro, INS_Data.Accel)
		rate.sleep()

	# IMU calculate and Kalman Filter calculate
	i = 0
	Error = numpy.zeros( (15, 1) )
	while not rospy.is_shutdown() and Flag:
		
		Pos, Vel, Ang, Cb2n, Error = INS_Cal.Update(INS_Data.Gyro, INS_Data.Accel, Error, 0.02)
		i = i + 1

		if i == 50:
			Error = KF_Cal.Kalman_Filter(Pos, Vel, Ang, Cb2n, INS_Data.Gyro, INS_Data.Accel, 50 / 50)
			print(Error)
			i = 0

		#rospy.loginfo("Vel output:%f, %f, %f", Vel[0], Vel[1], Vel[2])
		#rospy.loginfo("Pos output:%f, %f, %f", Pos[0], Pos[1], Pos[2])
		#rospy.loginfo("Ang output:%f, %f, %f", Ang[0] * 180 / INS.pi, Ang[1] * 180 / INS.pi, Ang[2] * 180 / INS.pi)
		
		rate.sleep()

	rospy.spin()

if __name__ == "__main__":
	main()