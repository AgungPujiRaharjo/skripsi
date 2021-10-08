#!/usr/bin/env python3

import rospy
import sys
from common.Kalman import KalmanAngle
from std_msgs.msg import String
from mpu6050 import mpu6050
import time
import math

rospy.init_node('imu_mpu6050',anonymous=False)
rate=rospy.Rate(10)
pub=rospy.Publisher('mpu_data',String,queue_size=1)

global kalmanX, kalmanY
kalmanX = KalmanAngle()
kalmanY = KalmanAngle()
radToDeg = 57.2957786

AcGy = [0. for k in range (6)] #inisialisasi nilai akselerometer dan gyroscope

def init_IMU():
    global kalmanX, kalmanY
    time.sleep(1)
    AcGy = ambilData()

    accX = AcGy[0]
    accY = AcGy[1]
    accZ = AcGy[2]

    print("acc x = ", round(accX,3), " acc y = ", round(accY,3), " acc z = ", round(accZ,3))

    roll = math.atan2(-accY,accZ) * radToDeg
    pitch = math.atan2(accX,accZ) * radToDeg

    print("Roll awal = ",roll)
    print("Pitch awal = ",pitch)

    kalmanX.setAngle(roll)
    kalmanY.setAngle(pitch)
            
    return roll, pitch
    
def ambilData():
    mpu = mpu6050(0x68)
    imu = [0. for k in range (6)]
    
    accel_data = mpu.get_accel_data()
    imu[0] = accel_data['x']
    imu[1] = accel_data['y']
    imu[2] = accel_data['z']

    gyro_data = mpu.get_gyro_data()
    imu[3] = gyro_data['x']
    imu[4] = gyro_data['y']
    imu[5] = gyro_data['z']
    
    return imu

def calibrate_IMU():
    zeroX, zeroY = 0, 0
    print('calibrating IMU...')
    for i in range(100):
        AcGy = ambilData()
        zeroX -= AcGy[3]
        zeroY -= AcGy[4]
        time.sleep(0.05)
    zeroX /= 100
    zeroY /= 100
    print('zeroX: ', zeroX, 'zeroY: ', zeroY)
    return zeroX, zeroY

def get_robot_ori(timer, zeroX, zeroY, kalAngleX, kalAngleY, init_roll, init_pitch): 
    global kalmanX, kalmanY   
    AcGy = ambilData()
    ## calibrate with robot's global frame orientation
    accX = AcGy[0]
    accY = AcGy[1]
    accZ = AcGy[2]
    gyroX = AcGy[3] + zeroX
    gyroY = AcGy[4] + zeroY
    gyroZ = AcGy[5]
    
    dt = time.time() - timer
    timer = time.time()

    roll = math.atan2(-accY,accZ) * radToDeg
    pitch = math.atan2(accX,accZ) * radToDeg
        
    # print("roll = ", round(roll, 3), " pitch = ", round(pitch, 3), " gyroX = ", round(gyroX, 3), " gyroY = ", round(gyroY, 3))

    gyroXRate = gyroY
    gyroYRate = gyroX
    
    if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
        kalmanX.setAngle(roll)
        complAngleX = roll
        kalAngleX   = roll
        gyroXAngle  = roll
    else:
        kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)
    if(abs(kalAngleX)>90):
        gyroYRate  = -gyroYRate
        kalAngleY  = kalmanY.getAngle(pitch,gyroYRate,dt)

    if((pitch < -90 and kalAngleY >90) or (pitch > 90 and kalAngleY < -90)):
        kalmanY.setAngle(pitch)
        complAngleY = pitch
        kalAngleY   = pitch
        gyroYAngle  = pitch
    else:
        kalAngleY = kalmanY.getAngle(pitch,gyroYRate,dt)

    if(abs(kalAngleY)>90):
        gyroXRate  = -gyroXRate
        kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)
        
    # if kalAngleX < 2:
    #     kalAngleX = 0
    # if kalAngleY < 2:
    #     kalAngleY = 0

 ##    print('Pitch  : %1f' %(kalAngleY),'rate    : %1f'%kalmanY.rate,'bias: %1f' %(kalmanY.bias),sep='\t')
 ##    print()
 ##    time.sleep(0.005) #kalau terlalu lambat, maka semakin lambat pula pasnya

    kalRoll = kalAngleX - init_roll
    kalPitch = kalAngleY - init_pitch

    if (kalRoll < -180):
        kalRoll = round(360 + kalRoll,3)
        if (kalPitch < -180):
            kalPitch = round(360 + kalPitch,3)
        else:
            if (kalPitch > 180):
                kalPitch = round(kalPitch - 360,3)
            else:
                kalPitch = round(kalPitch,3)

    if (kalRoll > 180):
        kalRoll = round(kalRoll - 360,3)
        if (kalPitch < -180):
            kalPitch = round(360 + kalPitch,3)
        else:
            if (kalPitch > 180):
                kalPitch = round(kalPitch - 360,3)
            else:
                kalPitch = round(kalPitch,3)
    else:
        kalRoll = round(kalRoll,3)
        if (kalPitch < -180):
            kalPitch = round(360 + kalPitch,3)
        else:
            if (kalPitch > 180):
                kalPitch = round(kalPitch - 360,3)
            else:
                kalPitch = round(kalPitch,3)
    # print("timer : ", timer, "kalPitch : ", kalPitch, "kalRoll : ", kalRoll)

    return kalRoll, kalPitch, timer

init_roll, init_pitch = init_IMU()
zeroX, zeroY = calibrate_IMU()
roll, pitch = 0, 0
timer = time.time()

while not rospy.is_shutdown():
    roll, pitch, timer = get_robot_ori(timer, zeroX, zeroY, roll, pitch, init_roll, init_pitch)

    string_msg=str(round(roll,3))+","+str(round(pitch,3))
    pub.publish(string_msg)
    rate.sleep()


# while 1:
#     roll, pitch, timer = get_robot_ori(timer, zeroX, zeroY, roll, pitch, init_roll, init_pitch)
#     print("roll = ", round(roll, 3) , " pitch = ", round(pitch, 3))