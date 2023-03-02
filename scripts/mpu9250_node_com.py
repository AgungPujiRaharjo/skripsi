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

	rospy.init_node('program', anonymous=False)
	rate = rospy.Rate(30)
	pub = rospy.Publisher('com', Vector3, queue_size=1)
	msg = Vector3()
	zCOM_def = 180

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
		print("Roll:{0} Pitch:{1} Yaw:{2} ".format(round(sensorfusion.roll,2), round(sensorfusion.pitch,2), round(sensorfusion.yaw,2)))
		print("comX:{0} comY:{1} comZ1:{2}".format(COM_x, COM_y, COM_z))
		print()
		msg.x = sensorfusion.roll
		msg.y = sensorfusion.pitch
		msg.z = sensorfusion.yaw
		
		pub.publish(msg)
		rate.sleep()
		time.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
