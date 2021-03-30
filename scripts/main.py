#!/usr/bin/env python3

#sudo chmod a+rw /dev/ttyUSB0
import rospy
import os
import time
import servo
import servo_handler as rh
from rosCall import *
from library.common.time_lib import *
from dynamixel_sdk import *
import pandas as pd
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


##-----------------------Posisi default 
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

# Set port baudrate
if robot.portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
    
# Enable Dynamixel#1 Torque
robot.cekServo(ADDR_AX_TORQUE_ENABLE,TORQUE_ENABLE)

# #program untuk duduk 
# # aDuduk=[291,734,267,740,453,568,355,654,524,516,183,861,477,590,507,485,516,516]
# # for i in range(len(aDuduk)):
# #     dxl[i].moveSync(aDuduk[i],2,type='reg')
# # robot.syncWrite()

# ##----------------------------------------gerakin default-------------------------------
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
####--------------------------------------------------------------------------------------

# ##=============================coba pola dinamis==============
# invers(robot,dxl,'ki',0,-3,20,1)
# invers(robot,dxl,'ka',0,-3,18,1)
# robot.syncWrite()
# wait(1.5)

# resCOM=COM(robot,dxl,'ki')
# comDef["x"],comDef["y"],comDef["z"]=resCOM[0],resCOM[1],resCOM[2]
# print("comDef",comDef)
# wait(0.1)

# firstStep=1
# xGoal=[4,4]
# n=0
# base=-1 # -1 base kaki kiri, 1 base kaki kanan
# tsmp=0.1 #waktu sampling
# tsup=2 #waktu total satu langkah
# lastStep=0
# firstMicros=micros()

# while(1):
#     t1=time.time()
#     wait(0.021) 
    
#     currentMicros=micros()
#     t=currentMicros-firstMicros

#     walkUpdate(robot,dxl,t,tsup,base,xGoal[n],firstStep,lastStep)
#     Control(robot,dxl,base,t)
#     t2=time.time()
#     print("t satu kali:",t2-t1)
#     # jika satu langkah telah berakhir
#     if t/1000000>=tsup: #+0.3: 
#         print("==========================langkah ke-"+ str(n+1)+" selesai===========================")
#         base=base*(-1) # switch kaki tumpu
        
#         firstMicros=micros() #perbaharui waktu dari awal (0 detik)
#         firstStep=0 # bukan awal langkah lagi      
#         n+=1
#         if n==(len(xGoal))-1:
#             # lastStep=1 
#             print("waw") 

#         if n>(len(xGoal))-1:
#             break

#-------------------------------default virtual---------------------------------
# for obj in dxl :
#     obj.moveSync(obj.default, 2,0,type='reg',read=0)
# ##-------------------------------------------------------------------------------

# # # # ##=============================coba pola dinamis virtual==============
# invers(robot,dxl,'ki',0,0,20,1)
# invers(robot,dxl,'ka',0,0,20,1)
# wait(1.5)

# resCOM=COM(robot,dxl,'ki',readAll_leg='virtual')
# comDef["x"],comDef["y"],comDef["z"]=resCOM[0],resCOM[1],resCOM[2]
# print("comDef",comDef)
# wait(0.1)

# firstStep=1
# lastStep=0
# firstMicros=micros()
# xGoal=[4]
# n=0
# base=-1 # -1 base kaki kiri, 1 base kaki kanan
# tsmp=0.1 #waktu sampling
# tsup=2 #waktu total satu langkah

# while(1):
#     wait(0.1)
        
#     currentMicros=micros()
#     t=currentMicros-firstMicros

#     walkUpdate(robot,dxl,t,tsup,base,xGoal[n],firstStep,lastStep,condition='virtual')
#     Control2(robot,dxl,base,t,condition='virtual')

#     # jika satu langkah telah berakhir
#     if t/1000000>=tsup: #+0.3: 
#         print("==========================langkah ke-"+ str(n+1)+" selesai===========================")
#         base=base*(-1) # switch kaki tumpu

#         firstMicros=micros() #perbaharui waktu dari awal (0 detik)
#         firstStep=0 # bukan awal langkah lagi      
#         n+=1
#         if n==(len(xGoal))-1:
#             # lastStep=1 
#             print("waw") 

#         if n>(len(xGoal))-1:
#             break

#==========================ambil data uji kemiringan robot (pitch)=========================
invers(robot,dxl,'ki',0,0,20,1)
invers(robot,dxl,'ka',0,0,18,1)
dxl[17].moveSync(-15,1)
dxl[8].moveSync(-14,1)
dxl[16].moveSync(-15,1)
robot.syncWrite()
wait(1.5)

t15=robot.readOne(15,data='float')
t16=robot.readOne(16,data='float')
t11=robot.readOne(11,data='float')
t12=robot.readOne(12,data='float')

angleData=[]
comX=[]
comY=[]
comZ=[]
imuRoll=[]
imuPitch=[]

angle=0
#------uji roll semakin positif
while(1):
    print("uji pitch semakin positif-----Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
    s16=robot.readOne(16,data='float')

    angleData.append(s16-t16)
    print("angle:",s16-t16)

    #kirim servo engkel pitch
    # dxl[14].moveSync(t15-angle,0.5,dxl[14].prevGoal,read=0) #servo 15 engkel
    dxl[15].moveSync(t16+angle,0.5,dxl[15].prevGoal,read=0) #servo 16 engkel
    dxl[10].moveSync(t11-angle,0.5,dxl[10].prevGoal,read=0) #servo 11 hip
    dxl[11].moveSync(t12+angle,0.5,dxl[11].prevGoal,read=0) #servo 12 hip
    robot.syncWrite()
    wait(0.5)

    #dapetin com
    x,y,z=COM(robot,dxl,'ki')
    comX.append(x)
    comY.append(y)
    comZ.append(z)

    #dapetin imu
    roll,pitch=getMpu()
    imuRoll.append(roll)
    imuPitch.append(pitch)
    
    angle=angle+1 #ditambah 1 drajat nih (semakin positif)
    
angle=0
invers(robot,dxl,'ki',0,0,20,1)
invers(robot,dxl,'ka',0,0,18,1)
robot.syncWrite()
wait(1.5)

t15=robot.readOne(15,data='float')
t16=robot.readOne(16,data='float')
t11=robot.readOne(11,data='float')
t12=robot.readOne(12,data='float')

while(1):
    print("uji pitch semakin negatif-----Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
    
    s16=robot.readOne(16,data='float')

    angleData.append(s16-t16)
    print("angle:",s16-t16)

    #kirim servo engkel pitch
    # dxl[14].moveSync(t15-angle,0.5,dxl[14].prevGoal,read=0) #servo 15 engkel
    dxl[15].moveSync(t16+angle,0.5,dxl[15].prevGoal,read=0) #servo 16 engkel
    dxl[10].moveSync(t11-angle,0.5,dxl[10].prevGoal,read=0) #servo 11 hip
    dxl[11].moveSync(t12+angle,0.5,dxl[11].prevGoal,read=0) #servo 12 hip
    robot.syncWrite()
    wait(0.5)

    #dapetin com
    x,y,z=COM(robot,dxl,'ki')
    comX.append(x)
    comY.append(y)
    comZ.append(z)

    #dapetin imu
    roll,pitch=getMpu()
    imuRoll.append(roll)
    imuPitch.append(pitch)
    
    angle=angle-1 #dikurang 1 drajat nih (semakin negatif)

#masukin semua data ke excel
df = pd.DataFrame({'sudut':angleData,'com X':comX,'com Y':comY,'com Z':comZ,'imu Roll':imuRoll,'imu Pitch':imuPitch})
df.to_excel('./src/program/data kemiringan robot pitch single support dengan hip (invers).xlsx', index=False)
print("sudah diinput ke excel")
#=========================================================================================


#==========================ambil data uji kemiringan robot (roll)=========================
# invers(robot,dxl,'ki',0,0,20,1)
# invers(robot,dxl,'ka',0,0,17,1)

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

# angle=-15
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
# invers(robot,dxl,'ka',0,0,18,1)
# robot.syncWrite()
# wait(1.5)

# t17=robot.readOne(17)
# t18=robot.readOne(18)
# t9=robot.readOne(9)
# t10=robot.readOne(10)

# angle=-15

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
    
#     angle=angle-1 #dikurang 1 drajat nih (semakin negatif)

# #masukin semua data ke excel
# df = pd.DataFrame({'sudut':angleData,'com X':comX,'com Y':comY,'com Z':comZ,'imu Roll':imuRoll,'imu Pitch':imuPitch})
# df.to_excel('./src/program/data kemiringan robot (roll) -14 dengan hip (invers) single support.xlsx', index=False)
# print("sudah diinput ke excel")
# #=========================================================================================
