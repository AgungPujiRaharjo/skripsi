#!/usr/bin/python3
import os
import sys
import time
import smbus
import numpy as np
from math import radians, sin, cos

from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman 
import rospy
from geometry_msgs.msg import Vector3

address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

imu.loadCalibDataFromFile("/home/ophelia/catkin_ws/src/program/scripts/calib.json")

sensorfusion = kalman.Kalman()
def main():
	imu.readSensor()
	imu.computeOrientation()
	sensorfusion.roll = imu.roll
	sensorfusion.pitch = imu.pitch
	sensorfusion.yaw = imu.yaw

	count = 0
	currTime = time.time()

	rospy.init_node('mpu9250_node', anonymous=False)
	rate = rospy.Rate(30)
	pub_1 = rospy.Publisher('com', Vector3, queue_size=1)
	pub_2 = rospy.Publisher('acc', Vector3, queue_size=1)
	pub_3 = rospy.Publisher('zmp', Vector3, queue_size=1)
	com = Vector3()
	acc = Vector3()
	zmp = Vector3()
	zCOM_def = 235

	while True:
		imu.readSensor()
		imu.computeOrientation()
		newTime = time.time()
		dt = newTime - currTime
		currTime = newTime
                
		sensorfusion.computeAndUpdateRollPitchYaw(imu.AccelVals[1], imu.AccelVals[0], imu.AccelVals[2]*(-1), imu.GyroVals[1], imu.GyroVals[0], imu.GyroVals[2]*(-1),\
													imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)

		COM_x = round(zCOM_def * sin(round(radians(sensorfusion.pitch),2)), 2)
		COM_y = round(zCOM_def * sin(round(radians(sensorfusion.roll),2)), 2)
		COM_z = round(zCOM_def * cos(round(radians(sensorfusion.pitch),2)) * cos(round(radians(sensorfusion.roll),2)), 2)
        
		# sensorfusion(imu.AccelVals[1], imu.AccelVals[0], imu.AccelVals[2]*(-1))
		ZMP_x = COM_x-((COM_z*imu.AccelVals[0])/((imu.AccelVals[2]*(-1))+9.81))
		ZMP_y = COM_y-((COM_z*imu.AccelVals[1])/((imu.AccelVals[2]*(-1))+9.81))
		ZMP_z = 0
		print('accX:{0} accY:{1} accZ:{2}'.format(imu.AccelVals[1], imu.AccelVals[0], imu.AccelVals[2]*(-1)))
		# print("Roll:{0} Pitch:{1} Yaw:{2} ".format(round(sensorfusion.roll,2), round(sensorfusion.pitch,2), round(sensorfusion.yaw,2)))
		# print("comX:{0} comY:{1} comZ1:{2}".format(COM_x, COM_y, COM_z))
		# print('ZMP-x:{0} ZMP-y:{1}'.format(ZMP_x, ZMP_y))
		print()
                
		com.x = COM_x
		com.y = COM_y
		com.z = COM_z
        
		acc.x = imu.AccelVals[1]
		acc.y = imu.AccelVals[0]
		acc.z = imu.AccelVals[2]*(-1)

		zmp.x = ZMP_x
		zmp.y = ZMP_y
		zmp.z = ZMP_z
		
		pub_1.publish(com)
		pub_2.publish(acc)
		pub_3.publish(zmp)
		rate.sleep()
		time.sleep(0.03)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
