#!/usr/bin/env python3

#sudo chmod a+rw /dev/ttyUSB0
#source devel/setup.bash
import rospy
import os
import time
import servo
import servo_handler as rh
from rosCall import *
from library.common.time_lib import *
from dynamixel_sdk import *
import pandas as pd
import numpy as np
import scipy.linalg
from scipy import signal
import matplotlib.pyplot as plt

# from listener import data
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

dxl = []
angleNow={}
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Control table address
ADDR_AX_TORQUE_ENABLE      = 24            # Control table address is different in Dynamixel model
ADDR_AX_GOAL_POSITION      = 30
ADDR_AX_PRESENT_POSITION   = 36
ADDR_AX_MOVING             = 46

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 2

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque


##-----------------------Posisi default-----------------------------------
dxl.append(servo.Servo(205,1,portHandler,packetHandler)) 
dxl.append(servo.Servo(818,2,portHandler,packetHandler)) 
dxl.append(servo.Servo(279,3,portHandler,packetHandler))
dxl.append(servo.Servo(744,4,portHandler,packetHandler))
dxl.append(servo.Servo(462,5,portHandler,packetHandler))
dxl.append(servo.Servo(561,6,portHandler,packetHandler))
dxl.append(servo.Servo(358,7,portHandler,packetHandler))
dxl.append(servo.Servo(658,8,portHandler,packetHandler)) #666
dxl.append(servo.Servo(516,9,portHandler,packetHandler)) 
dxl.append(servo.Servo(516,10,portHandler,packetHandler))
dxl.append(servo.Servo(508,11,portHandler,packetHandler)) 
dxl.append(servo.Servo(508,12,portHandler,packetHandler)) 
dxl.append(servo.Servo(513,13,portHandler,packetHandler)) 
dxl.append(servo.Servo(513,14,portHandler,packetHandler)) 
dxl.append(servo.Servo(518,15,portHandler,packetHandler))  #515
dxl.append(servo.Servo(502,16,portHandler,packetHandler)) #505
dxl.append(servo.Servo(516,17,portHandler,packetHandler)) 
dxl.append(servo.Servo(516,18,portHandler,packetHandler))


# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_AX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
robot = rh.Robot(dxl,portHandler,packetHandler,groupSyncWrite)

# # Open port
if robot.portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# # # Set port baudrate
if robot.portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
    
# # Enable Dynamixel#1 Torque
robot.cekServo(ADDR_AX_TORQUE_ENABLE,TORQUE_ENABLE)

##----------------------------------------gerakin default-------------------------------
while 1:
    print("=========default====================")
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
        
    for obj in dxl :
        obj.moveSync(obj.default, 1.5,0,'reg')
    start = time.time()
    
    robot.syncWrite()
    indexMoving = 3
    while indexMoving > 2 :
        isMoving = robot.readAll(ADDR_AX_MOVING,1)
        print('isMoving: ', isMoving)
        indexMoving = 0
        for i in isMoving:
            indexMoving = indexMoving + i
    end = time.time()
    print("waktu: ", end - start)
###--------------------------------------------------------------------------------------

#-----------------------------------coba pola dengan LQR---------------------------------
# invers(robot,dxl,'ki',0,0,20,1)
# invers(robot,dxl,'ka',0,0,20,1)
# robot.syncWrite()
# wait(1.5)

# resCOM=COM(robot,dxl,'ki')
# comDef["x"],comDef["y"],comDef["z"],comDef["zt"]=resCOM[0],resCOM[1],resCOM[2],resCOM[2]
# print("comDef",comDef)
# wait(0.1)

# state1Roll=arctan(comNow["y"]/comNow["z"]) #masih dalam radian
# state1Pitch=arctan(comNow["x"]/comNow["z"])
# controlDict["rollBef"],controlDict["pitchBef"]=state1Roll,state1Pitch

# while(1):
#     inp=input("========press 'enter' to walk, 'i' to init invers = ")
#     if inp=="":
#         break

#     elif inp=="i" or inp=="I":
#         invers(robot,dxl,'ki',0,0,20,1)
#         invers(robot,dxl,'ka',0,0,20,1)
#         robot.syncWrite()
#         wait(1.5)

#         resCOM=COM(robot,dxl,'ki')
#         comDef["x"],comDef["y"],comDef["z"],comDef["zt"]=resCOM[0],resCOM[1],resCOM[2],resCOM[2]
#         print("comDef",comDef)

#         state1Roll=arctan(comNow["y"]/comNow["z"]) #masih dalam radian
#         state1Pitch=arctan(comNow["x"]/comNow["z"])
#         controlDict["rollBef"],controlDict["pitchBef"]=state1Roll,state1Pitch

# firstStep=1
# xGoal=[2,2,2,2]
# n=0
# base=-1 # -1 base kaki kiri, 1 base kaki kanan
# tsmp=0.1 #waktu sampling
# tsup=2 #waktu total satu langkah
# lastStep=0
# Q,K=tuningLQRdiskrit('walk') #tuning LQR untuk mendapatkan nilai K

# allPttrnXt=[]
# allPttrnYt=[]
# allCOMx=[]
# allCOMy=[]
# allCOMz=[]
# allTime=[]
# QSave=["Q"]
# KSave=["K"]
# tp=0
# fwdDef["x"]=fwdNow["x"]


# firstMicros=micros()
# while(1):
#     t1=time.time()
#     currentMicros=micros()
#     t=currentMicros-firstMicros

#     walkUpdate3(robot,dxl,t,tsup,base,xGoal[n],firstStep,lastStep)
#     Control3(robot,dxl,base,t,K)

#     allPttrnXt.append(pttrn["Xt"])
#     allPttrnYt.append(pttrn["Yt"])
#     allCOMx.append(comNow["x"])
#     allCOMy.append(comNow["y"])
#     allCOMz.append(comNow["z"])
#     allTime.append((t/1000)+tp)
#     t2=time.time()

#     wait(0.021)

#     print("t aksi:",t2-t1)
#     # jika satu langkah telah berakhir
#     if t/1000000>=tsup: #+0.3: 
#         print("==========================langkah ke-"+ str(n+1)+" selesai===========================")
#         base=base*(-1) # switch kaki tumpu

#         #forward dengan membaca semua servo
#         if base==-1:
#             res=forward(robot,dxl,'ki')
#             fwdDef["x"]=res[0]
#             print("fwd last",res)
           
#         elif base==1:
#             res=forward(robot,dxl,'ka')
#             fwdDef["x"]=res[0]
#             print("fwd last",res)

#         wait(0.15) 
#         firstMicros=micros() #perbaharui waktu dari awal (0 detik)
#         firstStep=0 # bukan awal langkah lagi      
#         n+=1
#         tp+=tsup*1000
#         if n==(len(xGoal))-1:
#             # lastStep=1 
#             print("waw") 

#         if n>(len(xGoal))-1:
#             break

# QSave.append(Q[0,0])
# QSave.append(Q[1,1])
# QSave.append(Q[2,2])
# QSave.append(Q[3,3])
# KSave.append(K[0,0])
# KSave.append(K[0,1])
# KSave.append(K[1,2])
# KSave.append(K[1,3])

# #masukin semua data ke excel
# df = pd.DataFrame({'waktu':allTime,'Xt':allPttrnXt,'Yt':allPttrnYt,'Zt':allCOMz,'COMx':allCOMx,'COMy':allCOMy,'COMz':allCOMz})
# filename="comRead_vs_refCOM"
# loc='./src/project_bioloid/program/data/data_com/%s.xlsx' % (filename)
# df.to_excel(loc, index=True)
# print("data diinput ke excel bernama : %s.xlsx" % filename)

# print("K",K)

# wb=load_workbook(loc)
# sh=wb.worksheets[0]

# for i in range(len(QSave)):
#     sh.cell(row=i+1,column=9,value=QSave[i])

# for i in range(len(KSave)):
#     sh.cell(row=i+1,column=10,value=KSave[i])

# wb.save(loc)
# # ##===============================================================

# -------------------------------default virtual---------------------------------
# for obj in dxl :
#     obj.moveSync(obj.default, 2,0,type='reg',read=0)
# # ##-------------------------------------------------------------------------------

#=============================coba pola LQR virtual==============
# invers(robot,dxl,'ki',0,0,20,1)
# invers(robot,dxl,'ka',0,0,20,1)
# wait(1.5)

# resCOM=COM(robot,dxl,'ki',readAll_leg='virtual')
# comDef["x"],comDef["y"],comDef["z"],comDef["zt"]=resCOM[0],resCOM[1],resCOM[2],resCOM[2]
# print("comDef",comDef)
# wait(0.1)

# state1Roll=arctan(comNow["y"]/comNow["z"]) #masih dalam radian
# state1Pitch=arctan(comNow["x"]/comNow["z"])
# controlDict["rollBef"],controlDict["pitchBef"]=state1Roll,state1Pitch

# while(1):
#     inp=input("========press 'enter' to walk, 'i' to init invers = ")
#     if inp=="":
#         break

#     elif inp=="i" or inp=="I":
#         invers(robot,dxl,'ki',0,0,20,1)
#         invers(robot,dxl,'ka',0,0,20,1)
#         wait(1.5)

#         resCOM=COM(robot,dxl,'ki',readAll_leg='virtual')
#         comDef["x"],comDef["y"],comDef["z"],comDef["zt"]=resCOM[0],resCOM[1],resCOM[2],resCOM[2]
#         print("comDef",comDef)

#         state1Roll=arctan(comNow["y"]/comNow["z"]) #masih dalam radian
#         state1Pitch=arctan(comNow["x"]/comNow["z"])
#         controlDict["rollBef"],controlDict["pitchBef"]=state1Roll,state1Pitch


# firstStep=1
# xGoal=[2,2,2,2]
# n=0
# base=-1 # -1 base kaki kiri, 1 base kaki kanan
# tsmp=0.1 #waktu sampling
# tsup=2 #waktu total satu langkah
# lastStep=0
# Q,K=tuningLQRdiskrit('walk') #tuning LQR untuk mendapatkan nilai K

# allPttrnXt=[]
# allPttrnYt=[]
# allCOMx=[]
# allCOMy=[]
# allCOMz=[]
# allTime=[]
# QSave=["Q"]
# KSave=["K"]
# tp=0
# fwdDef["x"]=fwdNow["x"]

# firstMicros=micros()
# while(1):

#     t1=time.time()
#     currentMicros=micros()
#     t=currentMicros-firstMicros

#     walkUpdate3(robot,dxl,t,tsup,base,xGoal[n],firstStep,lastStep,condition='virtual')
#     Control3(robot,dxl,base,t,K,condition='virtual')

#     allPttrnXt.append(pttrn["Xt"])
#     allPttrnYt.append(pttrn["Yt"])
#     allCOMx.append(comNow["x"])
#     allCOMy.append(comNow["y"])
#     allCOMz.append(comNow["z"])
#     allTime.append((t/1000)+tp)
#     t2=time.time()

#     wait(0.021)

#     print("t aksi:",t2-t1)
#     # jika satu langkah telah berakhir
#     if t/1000000>=tsup: #+0.3: 
#         print("==========================langkah ke-"+ str(n+1)+" selesai===========================")
#         base=base*(-1) # switch kaki tumpu

#         #forward dengan membaca semua servo
#         if base==-1:
#             res=forward(robot,dxl,'ki',readAll_leg='virtual')
#             fwdDef["x"]=res[0]
#             print("fwd last",res)
           
#         elif base==1:
#             res=forward(robot,dxl,'ka',readAll_leg='virtual')
#             fwdDef["x"]=res[0]
#             print("fwd last",res)

#         wait(0.5)
#         firstMicros=micros() #perbaharui waktu dari awal (0 detik)
#         firstStep=0 # bukan awal langkah lagi      
#         n+=1
#         tp+=tsup*1000
#         if n==(len(xGoal))-1:
#             # lastStep=1 
#             print("waw") 

#         if n>(len(xGoal))-1:
#             break

# QSave.append(Q[0,0])
# QSave.append(Q[1,1])
# QSave.append(Q[2,2])
# QSave.append(Q[3,3])
# KSave.append(K[0,0])
# KSave.append(K[0,1])
# KSave.append(K[1,2])
# KSave.append(K[1,3])

# ###-------------------------------data control LQR (com)--------------------------------
# #masukin semua data ke excel
# df = pd.DataFrame({'waktu':allTime,'Xt':allPttrnXt,'Yt':allPttrnYt,'Zt':allCOMz,'COMx':allCOMx,'COMy':allCOMy,'COMz':allCOMz})
# filename="comRead_vs_refCOM_virtual"
# loc='./src/project_bioloid/program/data/data_com/%s.xlsx' % (filename)
# df.to_excel(loc, index=True)
# print("data diinput ke excel bernama : %s.xlsx" % filename)

# print("K",K)

# wb=load_workbook(loc)
# sh=wb.worksheets[0]

# for i in range(len(QSave)):
#     sh.cell(row=i+1,column=9,value=QSave[i])

# for i in range(len(KSave)):
#     sh.cell(row=i+1,column=10,value=KSave[i])

# wb.save(loc)
##              -----------------------------------------------------------------------------------------

# #------------------------------------plot com----------------------------------
# fig, axs = plt.subplots(2)
# fig.tight_layout(pad=2.0)

# #--------jika ingin grafik dalam bentuk com-----------
# axs[0].set_title('sumbu X',loc="left")
# axs[0].plot(allTime,allPttrnXt,"g",label='COM Referensi')
# axs[0].plot(allTime,allCOMx,"r",label="COM Dibaca")
# axs[0].legend(loc="upper left")
# axs[0].grid()
# axs[0].set_xlabel("time (ms)")
# axs[0].set_ylabel("COM X (mm)")

# axs[1].set_title('sumbu Y',loc="left")
# axs[1].plot(allTime,allPttrnYt,"g",label='COM Referensi')
# axs[1].plot(allTime,allCOMy,"r",label="COM Dibaca")
# axs[1].legend(loc="upper left")
# axs[1].grid()
# axs[1].set_xlabel("time (ms)")
# axs[1].set_ylabel("COM Y (mm)")
# #--------------------------------------------------------

# plt.savefig('./src/program/data/data plot com vs ref com.png')
# ##======================================================================

#==========================ambil data uji kemiringan robot (pitch)=========================
# invers(robot,dxl,'ki',0,0,18,1)
# invers(robot,dxl,'ka',0,0,20,1)
# dxl[16].moveSync(15,1) #servo17
# dxl[9].moveSync(14,1)  #servo10
# dxl[17].moveSync(15,1)
# # dxl[17].moveSync(-15,1)
# # dxl[8].moveSync(-14,1)
# # dxl[16].moveSync(-15,1) #15
# robot.syncWrite()
# wait(1.5)

# t15=robot.readOne(15)#,data='float')
# t16=robot.readOne(16)#,data='float')
# t11=robot.readOne(11)#,data='float')
# t12=robot.readOne(12)#,data='float')

# angleData=[]
# comX=[]
# comY=[]
# comZ=[]
# imuRoll=[]
# imuPitch=[]

# angle=0
# #------uji roll semakin positif
# while(1):
#     print("uji pitch semakin positif-----Press any key to continue! (or press ESC to quit!)")
#     if getch() == chr(0x1b):
#         break
#     # s16=robot.readOne(16,data='float')

#     angleData.append(angle)#s16-t16)
#     print("angle:",angle)#s16-t16)

#     #kirim servo engkel pitch
#     dxl[14].moveSync(t15-angle,0.5,dxl[14].prevGoal,read=0) #servo 15 engkel
#     dxl[15].moveSync(t16+angle,0.5,dxl[15].prevGoal,read=0) #servo 16 engkel
#     dxl[10].moveSync(t11-angle,0.5,dxl[10].prevGoal,read=0) #servo 11 hip
#     dxl[11].moveSync(t12+angle,0.5,dxl[11].prevGoal,read=0) #servo 12 hip
#     robot.syncWrite()
#     wait(0.5)

#     #dapetin com
#     x,y,z=COM(robot,dxl,'ki')
#     comX.append(x)
#     comY.append(y)
#     comZ.append(z)

#     #dapetin imu
#     roll,pitch=getMpu()
#     imuRoll.append(roll)
#     imuPitch.append(pitch)
    
#     angle=angle+1 #ditambah 1 drajat nih (semakin positif)
    
# angle=0
# invers(robot,dxl,'ki',0,0,18,1)
# invers(robot,dxl,'ka',0,0,20,1)
# dxl[16].moveSync(15,1) #servo17
# dxl[9].moveSync(14,1)  #servo10
# dxl[17].moveSync(15,1)
# # dxl[17].moveSync(-15,1) #servo18
# # dxl[8].moveSync(-14,1)  #servo9
# # dxl[16].moveSync(-15,1) #17
# robot.syncWrite()
# wait(1.5)

# t15=robot.readOne(15)#,data='float')
# t16=robot.readOne(16)#,data='float')
# t11=robot.readOne(11)#,data='float')
# t12=robot.readOne(12)#,data='float')

# while(1):
#     print("uji pitch semakin negatif-----Press any key to continue! (or press ESC to quit!)")
#     if getch() == chr(0x1b):
#         break
    
#     # s16=robot.readOne(16,data='float')

#     angleData.append(angle)#s16-t16)
#     print("angle:",angle)#s16-t16)

#     #kirim servo engkel pitch
#     dxl[14].moveSync(t15-angle,0.5,dxl[14].prevGoal,read=0) #servo 15 engkel
#     dxl[15].moveSync(t16+angle,0.5,dxl[15].prevGoal,read=0) #servo 16 engkel
#     dxl[10].moveSync(t11-angle,0.5,dxl[10].prevGoal,read=0) #servo 11 hip
#     dxl[11].moveSync(t12+angle,0.5,dxl[11].prevGoal,read=0) #servo 12 hip
#     robot.syncWrite()
#     wait(0.5)

#     #dapetin com
#     x,y,z=COM(robot,dxl,'ki')
#     comX.append(x)
#     comY.append(y)
#     comZ.append(z)

#     #dapetin imu
#     roll,pitch=getMpu()
#     imuRoll.append(roll)
#     imuPitch.append(pitch)
    
#     angle=angle-1 #dikurang 1 derajat(semakin negatif)

# #masukin semua data ke excel
# df = pd.DataFrame({'sudut':angleData,'com X':comX,'com Y':comY,'com Z':comZ,'imu Roll':imuRoll,'imu Pitch':imuPitch})
# df.to_excel('./src/program/data/data pitch kemiringan robot single support kanan base.xlsx', index=False)
# print("sudah diinput ke excel")
# #=========================================================================================


# ==========================ambil data uji kemiringan robot (roll)=========================

# invers(robot,dxl,'ki',0,0,20,1)
# invers(robot,dxl,'ka',0,0,17,1) #17
# dxl[17].moveSync(-19,1)
# dxl[8].moveSync(-14,1)
# dxl[16].moveSync(-17,1) #15
# robot.syncWrite()
# wait(1.5)

# t17=robot.readOne(17)
# t18=robot.readOne(18)
# t9=robot.readOne(9)
# t10=robot.readOne(10)

# angleData=[]
# comX=[]
# comY=[]
# comZ=[]
# imuRoll=[]
# imuPitch=[]

# angle=0 #-15
# #------uji roll semakin positif
# while(1):
#     print("uji roll semakin positif-----Press any key to continue! (or press ESC to quit!)")
#     if getch() == chr(0x1b):
#         break
    
#     angleData.append(angle)
#     print("angle:",angle)

#     #kirim servo engkel roll
#     dxl[16].moveSync(t17+angle,0.5,dxl[16].prevGoal,read=0) #servo 17
#     dxl[17].moveSync(t18+angle,0.5,dxl[17].prevGoal,read=0) #servo 18
#     dxl[8].moveSync(t9+angle,0.5,dxl[8].prevGoal,read=0) #servo 9 hip
#     # dxl[9].moveSync(t10+angle,0.5,dxl[9].prevGoal,read=0) #servo 10 hip
#     robot.syncWrite()
#     wait(0.5)

#     #dapetin com
#     x,y,z=COM(robot,dxl,'ki')
#     comX.append(x)
#     comY.append(y)
#     comZ.append(z)

#     #dapetin imu
#     roll,pitch=getMpu()
#     imuRoll.append(roll)
#     imuPitch.append(pitch)
    
#     angle=angle+1 #ditambah 1 drajat nih (semakin positif)
    
# invers(robot,dxl,'ki',0,0,20,1)
# invers(robot,dxl,'ka',0,0,17,1)
# dxl[17].moveSync(-19,1)
# dxl[8].moveSync(-14,1)
# dxl[16].moveSync(-18,1) #15
# robot.syncWrite()
# wait(1.5)

# t17=robot.readOne(17)
# t18=robot.readOne(18)
# t9=robot.readOne(9)
# t10=robot.readOne(10)

# angle=0 #-15

# while(1):
#     print("uji roll semakin negatif-----Press any key to continue! (or press ESC to quit!)")
#     if getch() == chr(0x1b):
#         break
    
#     angleData.append(angle)
#     print("angle:",angle)

#     #kirim servo engkel roll
#     dxl[16].moveSync(t17+angle,0.5,dxl[16].prevGoal,read=0) #servo 17
#     dxl[17].moveSync(t18+angle,0.5,dxl[17].prevGoal,read=0) #servo 18
#     dxl[8].moveSync(t9+angle,0.5,dxl[8].prevGoal,read=0) #servo 9 hip
#     # dxl[9].moveSync(t10+angle,0.5,dxl[9].prevGoal,read=0) #servo 10 hip dimattin kalau single
#     robot.syncWrite()
#     wait(0.5)

#     #dapetin com
#     x,y,z=COM(robot,dxl,'ki')
#     comX.append(x)
#     comY.append(y)
#     comZ.append(z)

#     #dapetin imu
#     roll,pitch=getMpu()
#     imuRoll.append(roll)
#     imuPitch.append(pitch)
    
#     angle=angle-1 #dikurang 1 drajat nih (semakin negatif)

# #masukin semua data ke excel
# df = pd.DataFrame({'sudut':angleData,'com X':comX,'com Y':comY,'com Z':comZ,'imu Roll':imuRoll,'imu Pitch':imuPitch})
# df.to_excel('./src/program/data/data kemiringan robot roll single support base kiri.xlsx', index=False)
# print("sudah diinput ke excel")
#=========================================================================================

# ##------------------------------------plot com----------------------------------
# fig, axs = plt.subplots(2)
# fig.tight_layout(pad=2.0)

# #--------jika ingin grafik dalam bentuk com-----------
# axs[0].set_title('sumbu X',loc="left")
# axs[0].plot(allTime,allPttrnXt,"r",label='COM Referensi')
# axs[0].plot(allTime,allCOMx,"y",label="COM Dibaca")
# axs[0].legend(loc="upper left")
# axs[0].grid()
# axs[0].set_xlabel("time (ms)")
# axs[0].set_ylabel("COM X (mm)")

# axs[1].set_title('sumbu Y',loc="left")
# axs[1].plot(allTime,allPttrnYt,"r",label='COM Referensi')
# axs[1].plot(allTime,allCOMy,"y",label="COM Dibaca")
# axs[1].legend(loc="lower right")
# axs[1].grid()
# axs[1].set_xlabel("time (ms)")
# axs[1].set_ylabel("COM Y (mm)")
# #--------------------------------------------------------

# plt.savefig('./src/program/data/data_plot com vs ref com.png')
# ###=========================================================

# ##=============================coba pola dinamis==============
# invers(robot,dxl,'ki',0,0,20,1)
# invers(robot,dxl,'ka',0,0,20,1)
# robot.syncWrite()
# wait(1.5)

# resCOM=COM(robot,dxl,'ki')
# comDef["x"],comDef["y"],comDef["z"],comDef["zt"]=resCOM[0],resCOM[1],resCOM[2],resCOM[2]
# print("comDef",comDef)
# wait(0.1)

# state1Roll=arctan(comNow["y"]/comNow["z"]) #masih dalam radian
# state1Pitch=arctan(comNow["x"]/comNow["z"])
# controlDict["rollBef"],controlDict["pitchBef"]=state1Roll,state1Pitch

# while(1):
#     inp=input("========press 'enter' to walk, 'i' to init invers = ")
#     if inp=="":
#         break

#     elif inp=="i" or inp=="I":
#         invers(robot,dxl,'ki',0,0,20,1)
#         invers(robot,dxl,'ka',0,0,20,1)
#         robot.syncWrite()
#         wait(1.5)

#         resCOM=COM(robot,dxl,'ki')
#         comDef["x"],comDef["y"],comDef["z"],comDef["zt"]=resCOM[0],resCOM[1],resCOM[2],resCOM[2]
#         print("comDef",comDef)

#         state1Roll=arctan(comNow["y"]/comNow["z"]) #masih dalam radian
#         state1Pitch=arctan(comNow["x"]/comNow["z"])
#         controlDict["rollBef"],controlDict["pitchBef"]=state1Roll,state1Pitch

# firstStep=1
# xGoal=[3,3]
# n=0
# base=-1 # -1 base kaki kiri, 1 base kaki kanan
# tsmp=0.1 #waktu sampling
# tsup=2 #waktu total satu langkah
# lastStep=0
# Q,K=tuningLQR('walk') #tuning LQR untuk mendapatkan nilai K
# print(K)

# allPttrnXt=[]
# allPttrnYt=[]
# allCOMx=[]
# allCOMy=[]
# allCOMz=[]
# allTime=[]
# QSave=["Q"]
# KSave=["K"]
# tp=0
# fwdDef["x"]=fwdNow["x"]

# allxBase=[]
# allxSwing=[]
# allxPattern=[]
# alltBase=[]
# allxFwd=[]

# firstMicros=micros()
# while(1):

#     t1=time.time()
#     currentMicros=micros()
#     t=currentMicros-firstMicros

#     walkUpdate3(robot,dxl,t,tsup,base,xGoal[n],firstStep,lastStep)
#     Control3(robot,dxl,base,t,K)

#     allPttrnXt.append(pttrn["Xt"])
#     allPttrnYt.append(pttrn["Yt"])
#     allCOMx.append(comNow["x"])
#     allCOMy.append(comNow["y"])
#     allCOMz.append(comNow["z"])
#     allTime.append((t/1000)+tp)

#     allxBase.append(swngPlan["xbase"])
#     allxSwing.append(swngPlan["xswing"])
#     allxPattern.append(swngPlan["sfx"])
#     alltBase.append(swngPlan["tbase"])
#     allxFwd.append(swngPlan["xFwd"])
#     t2=time.time()

#     wait(0.021)

#     print("t aksi:",t2-t1)
#     # jika satu langkah telah berakhir
#     if t/1000000>=tsup +0.09: 
#         print("==========================langkah ke-"+ str(n+1)+" selesai===========================")
#         base=base*(-1) # switch kaki tumpu

#         #forward dengan membaca semua servo
#         if base==-1:
#             resCOM=COM(robot,dxl,'ki')
#             pttrn["comXinit"]=resCOM[0]
#             res=forward(robot,dxl,'ki')
#             fwdDef["x"]=res[0]
#             print("fwd last",res)
           
#         elif base==1:
#             resCOM=COM(robot,dxl,'ka')
#             pttrn["comXinit"]=resCOM[0]
#             res=forward(robot,dxl,'ka')
#             fwdDef["x"]=res[0]
#             print("fwd last",res)

#         wait(0.5)
#         firstMicros=micros() #perbaharui waktu dari awal (0 detik)
#         firstStep=0 # bukan awal langkah lagi      
#         n+=1
#         tp+=tsup*1000
#         if n==(len(xGoal))-1:
#             # lastStep=1 
#             print("waw") 

#         if n>(len(xGoal))-1:
#             break

# QSave.append(Q[0,0])
# QSave.append(Q[1,1])
# QSave.append(Q[2,2])
# QSave.append(Q[3,3])
# KSave.append(K[0,0])
# KSave.append(K[0,1])
# KSave.append(K[1,2])
# KSave.append(K[1,3])

# ###                 -------------------------------data control LQR (com)--------------------------------
# #masukin semua data ke excel
# df = pd.DataFrame({'waktu':allTime,'Xt':allPttrnXt,'Yt':allPttrnYt,'Zt':allCOMz,'COMx':allCOMx,'COMy':allCOMy,'COMz':allCOMz})
# filename="comRead_vs_refCOM"
# loc='./src/project_bioloid/program/data/data_com/%s.xlsx' % (filename)
# df.to_excel(loc, index=True)
# print("data diinput ke excel bernama : %s.xlsx" % filename)

# print("K",K)

# wb=load_workbook(loc)
# sh=wb.worksheets[0]

# for i in range(len(QSave)):
#     sh.cell(row=i+1,column=9,value=QSave[i])

# for i in range(len(KSave)):
#     sh.cell(row=i+1,column=10,value=KSave[i])

# wb.save(loc)
# ##              -----------------------------------------------------------------------------------------


# #==================ambil data invers kinematik=================
# disGoal=[]
# disX=[]
# disMsrX=[]

# while(1):
    
#     inp=float(input("berapa jarak :"))
    
#     while 1:
#         jarak=inp/2
#         invers(robot,dxl,'ki',jarak,0,20,1)
#         invers(robot,dxl,'ka',-jarak,0,20,1)
#         robot.syncWrite()
#         wait(1.2)

#         x,y,z=forward(robot,dxl,'ki')
#         print("x",x/10)
#         # print("y",y)
#         # print("z",z)

#         print("kirim lagi ? (or press ESC to quit!)")
#         if getch() == chr(0x1b):
#             break
    
#     disX.append(abs(x/10))
#     inp2=float(input("input ukur asli: "))
#     disMsrX.append(inp2)
#     disGoal.append(inp)

#     print("input lagi? (or press ESC to quit!)")
#     if getch() == chr(0x1b):
#         break

# #masukin semua data ke excel
# df = pd.DataFrame({'jarak tujuan':disGoal,'jarak forward':disX,'jarak ukur':disMsrX})
# filename="data ukur invers"
# loc='./src/project_bioloid/program/data/data_ukur_jarak/%s.xlsx' % (filename)
# df.to_excel(loc, index=False)
# print("data diinput ke excel bernama : %s.xlsx" % filename)

# ##================================uji translasi roll ===============================
allPttrnXt=[]
allPttrnYt=[]
allPttrnZt=[]
allCOMx=[]
allCOMy=[]
allCOMz=[]
allTime=[]
QSave=["Q"]
KSave=["K"]

invers(robot,dxl,'ki',0,0,20,1)
invers(robot,dxl,'ka',0,0,20,1)
robot.syncWrite()
wait(1.2)

t9def=dxl[8].prevGoalDegree
t10def=dxl[9].prevGoalDegree
t17def=dxl[16].prevGoalDegree
t18def=dxl[17].prevGoalDegree

resCOM=COM(robot,dxl,'ki')
comDef["x"],comDef["y"],comDef["z"],comDef["zt"]=resCOM[0],resCOM[1],resCOM[2],resCOM[2]
print("comDef",comDef)
wait(0.1)

tawal=-13
dxl[8].moveSync(t9def-tawal,1) #servo 9
dxl[9].moveSync(t10def-tawal,1) #servo 10
dxl[16].moveSync(t17def-tawal,1) #servo 17
dxl[17].moveSync(t18def-tawal,1) #servo 18
robot.syncWrite()
wait(1.5)

state1Roll=arctan(comNow["y"]/comNow["z"]) #masih dalam radian
state1Pitch=arctan(comNow["x"]/comNow["z"])
controlDict["rollBef"],controlDict["pitchBef"]=state1Roll,state1Pitch
wait(0.1)

tsmp=0.1 #waktu sampling
tsup=2 #waktu 

Q,K=tuningLQRdiskrit('translation roll')
pttrn["Yt"]=comDef["y"]
pttrn["Zt"]=comDef["zt"]
base='ki'

firstMicros=micros()
while(1):
    
    print("ref comY",pttrn["Yt"])
    # t1=time.time()
    currentMicros=micros()
    t=currentMicros-firstMicros
    # print("waktu",t)

    t1=time.time()
    cntTransRoll(robot,dxl,base,K,t)
    # cntTransRoll(robot,dxl,base,K,t,condition='virtual')

    allPttrnXt.append(pttrn["Xt"])
    allPttrnYt.append(pttrn["Yt"])
    allPttrnZt.append(pttrn["Zt"])
    allCOMx.append(comNow["x"])
    allCOMy.append(comNow["y"])
    allCOMz.append(comNow["z"])
    allTime.append(t/1000)
    t2=time.time()
    print("tot",t2-t1)
    wait(0.03)

    if t/1000000>=tsup:
        break

QSave.append(Q[0,0])
QSave.append(Q[1,1])
QSave.append(Q[2,2])
QSave.append(Q[3,3])
KSave.append(K[0,0])
KSave.append(K[0,1])
KSave.append(K[1,2])
KSave.append(K[1,3])

#masukin semua data ke excel
df = pd.DataFrame({'waktu':allTime,'Xt':allPttrnXt,'Yt':allPttrnYt,'Zt':allPttrnZt,'COMx':allCOMx,'COMy':allCOMy,'COMz':allCOMz})
filename="tuning COM translasi roll"
loc='./src/program/data/%s.xlsx' % (filename)
df.to_excel(loc, index=True)
print("data diinput ke excel bernama : %s.xlsx" % filename)

print("Q",Q)
print("K",K)

plt.plot(allTime,allPttrnYt,"g",label='COM Referensi')
plt.plot(allTime,allCOMy,"r",label='COM Dibaca')
plt.grid()
plt.xlabel("time (ms)")
plt.ylabel("COM Y (mm)")
plt.show()
plt.savefig('./src/program/data/data translasi roll.png')


# # ##================================uji translasi pitch ===============================
# allPttrnXt=[]
# allPttrnYt=[]
# allPttrnZt=[]
# allCOMx=[]
# allCOMy=[]
# allCOMz=[]
# allTime=[]

# invers(robot,dxl,'ki',0,0,20,1)
# invers(robot,dxl,'ka',0,0,20,1)
# robot.syncWrite()
# wait(1.2)

# t16def=dxl[15].prevGoalDegree
# t15def=dxl[14].prevGoalDegree
# t12def=dxl[11].prevGoalDegree
# t11def=dxl[10].prevGoalDegree
# # print("t16",t16def)
# # print("t15",t15def)
# # print("t12",t12def)
# # print("t15",t11def)

# resCOM=COM(robot,dxl,'ki')
# # resCOM=COM(robot,dxl,'ki',readAll_leg='virtual')
# comDef["x"],comDef["y"],comDef["z"],comDef["zt"]=resCOM[0],resCOM[1],resCOM[2],resCOM[2]
# print("comDef",comDef)
# wait(0.1)


# tawal=-7 #-13
# dxl[15].moveSync(t16def+tawal,1) #servo 16
# dxl[14].moveSync(t15def-tawal,1) #servo 15
# dxl[11].moveSync(t12def-tawal,1) #servo 12
# dxl[10].moveSync(t11def+tawal,1) #servo 11
# robot.syncWrite()
# wait(1.5)

# # dxl[15].moveSync(t16def+tawal,1,dxl[15].prevGoal,read=0) #servo 16
# # dxl[14].moveSync(t15def-tawal,1,dxl[14].prevGoal,read=0) #servo 15
# # dxl[11].moveSync(t12def-tawal,1,dxl[11].prevGoal,read=0) #servo 12
# # dxl[10].moveSync(t11def+tawal,1,dxl[10].prevGoal,read=0) #servo 11
# # robot.syncWrite()
# # wait(1.5)

# # resCOM=COM(robot,dxl,'ki',readAll_leg='virtual')
# resCOM=COM(robot,dxl,'ki')
# state1Roll=arctan(comNow["y"]/comNow["z"]) #masih dalam radian
# state1Pitch=arctan(comNow["x"]/comNow["z"])
# controlDict["rollBef"],controlDict["pitchBef"]=state1Roll,state1Pitch
# wait(0.1)

# tsmp=1 #waktu sampling
# tsup=2 #waktu 

# # K=array([[6.78371241,1.49951986,0,0], #roll
# #         [0,0,4,1.47693166]]) #pitch
# Q,K=tuningLQRdiskrit('translation pitch')
# pttrn["Xt"]=comDef["x"]
# pttrn["Zt"]=comDef["zt"]
# base='ki'

# firstMicros=micros()
# while(1):
    
#     print("ref comX",pttrn["Xt"])
#     # t1=time.time()
#     currentMicros=micros()
#     t=currentMicros-firstMicros
#     # print("waktu",t)

#     t1=time.time()
#     # cntTransPitch(robot,dxl,base,K,t)
#     cntTransPitch(robot,dxl,base,K,t,condition='normal')

#     allPttrnXt.append(pttrn["Xt"])
#     allPttrnYt.append(pttrn["Yt"])
#     allPttrnZt.append(pttrn["Zt"])
#     allCOMx.append(comNow["x"])
#     allCOMy.append(comNow["y"])
#     allCOMz.append(comNow["z"])
#     allTime.append(t/1000)
#     t2=time.time()
#     print("tot",t2-t1)
#     wait(0.03)

#     if t/1000000>=tsup:
#         break
    
# #masukin semua data ke excel
# df = pd.DataFrame({'waktu':allTime,'Xt':allPttrnXt,'Yt':allPttrnYt,'Zt':allPttrnZt,'COMx':allCOMx,'COMy':allCOMy,'COMz':allCOMz})
# filename="tuning COM translasi pitch"
# loc='./src/program/data/%s.xlsx' % (filename)
# df.to_excel(loc, index=True)
# print("data diinput ke excel bernama : %s.xlsx" % filename)

# print("Q",Q)
# print("K",K)

# plt.plot(allTime,allPttrnXt,"g",label='COM Referensi')
# plt.plot(allTime,allCOMx,"r",label='COM Dibaca')
# plt.grid()
# plt.xlabel("time (ms)")
# plt.ylabel("COM X (mm)")
# plt.show()
# plt.savefig('./src/program/data/data translasi pitch.png')

# # ==============================================