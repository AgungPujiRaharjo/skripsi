#!/usr/bin/env python3

import rospy
import time
from program.srv import *
from std_msgs.msg import String
from dynamixel_sdk_examples.srv import *
import servo
from numpy import sin,cos,tan,pi,array,eye,dot,arccos,arcsin,arctan,radians,degrees
import math
import numpy as np
import control as ctl
import scipy.linalg
from scipy import signal

rospy.init_node("rosCall_node",anonymous=False)
imuData={"roll":0,"pitch":0,"yaw":0,"refDir":0}
comDef={"x":0,"y":0,"z":0}
invPttrn={"la_Ki":0,"la_Ka":0,"t9":0,"t10":0,"t15":0,"t16":0,"t17":0,"t18":0}
controlDict={"rollBef":0.0,"pitchBef":0.0,"IAEX":0,"IAEY":0}
pttrn={"Xt":0,"Yt":0,"Zt":0,"sfx":0,"sfy":0,"sfz":0,"xS":0,"xNow:":0}
fwdNow={"x":0,"y":0,"z":0}
fwdDef={"x":0}
comNow={"x":0,"y":0,"z":0}
comXPolaPeriod=[0,0,0,0,0]
comYPolaPeriod=[0,0,0,0,0]


def getMagneto(dataFlag='all'):

    IMU_magneto=rospy.ServiceProxy('get_magneto_data',GetMagnetoData)
    req=GetMagnetoDataRequest()
    req.what_data.data=dataFlag
    resp=IMU_magneto.call(req)
    
    return resp.magneto_data.data

def getUltrasonic(dataFlag='all'):
    sensor_ultrasonic=rospy.ServiceProxy('get_ultrasonic_data',GetUltrasonicData)
    req=GetUltrasonicDataRequest()
    req.what_data.data=dataFlag
    resp=sensor_ultrasonic.call(req)

    return resp.ultrasonic_data.data

def getMpu(dataFlag='all'):
    IMU_mpu=rospy.ServiceProxy('get_mpu_data',GetMpuData)
    req=GetMpuDataRequest()
    req.what_data.data=dataFlag
    resp=IMU_mpu.call(req)
    mpu = [float(x) for x in resp.mpu_data.data.split(",")]
    imuData["roll"],imuData["pitch"]=mpu[0],mpu[1]

    return mpu[0],mpu[1]

def rosReadServo(idDxl):
    rospy.wait_for_service('get_position')
    read_angle=rospy.ServiceProxy('get_position',GetPosition)
    req=GetPositionRequest()
    req.id=idDxl
    resp=read_angle.call(req)
    result=resp.position
    print("sudut",result)

    return result

def invers(robot,dxl,base,x,y,z,time):

    IK=rospy.ServiceProxy('compute_invers_kinematics',ComputeInversKinematics)
    req=ComputeInversKinematicsRequest()
    req.base.data=base
    req.coordinatX.data=float(x)
    req.coordinatY.data=float(y)
    req.coordinatZ.data=float(z)

    resp=IK.call(req)
    angle = [float(x) for x in resp.angleServo.data.split(",")]

    if base=='ki':
        t7,t10,t12,t14,t16,t18=angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]
        dxl[6].moveSync(t7,time,dxl[7].prevGoal,read=0)
        dxl[9].moveSync(t10,time,dxl[9].prevGoal,read=0)
        dxl[11].moveSync(t12,time,dxl[11].prevGoal,read=0)
        dxl[13].moveSync(t14,time,dxl[13].prevGoal,read=0)
        dxl[15].moveSync(t16,time,dxl[15].prevGoal,read=0)
        dxl[17].moveSync(t18,time,dxl[17].prevGoal,read=0)
        # robot.syncWrite()
   
        invPttrn["la_Ki"],invPttrn["t16"],invPttrn["t18"]=resp.lengthLA.data,t16,t18
        
    elif base=='ka':
        t8,t9,t11,t13,t15,t17=angle[0],angle[1],angle[2],angle[3],angle[4],angle[5]

        dxl[7].moveSync(t8,time,dxl[6].prevGoal,read=0)
        # dxl[8].moveSync(t9,time,dxl[8].prevGoal,read=0)
        dxl[10].moveSync(t11,time,dxl[10].prevGoal,read=0)
        dxl[12].moveSync(t13,time,dxl[12].prevGoal,read=0)
        dxl[14].moveSync(t15,time,dxl[14].prevGoal,read=0)
        dxl[16].moveSync(t17,time,dxl[16].prevGoal,read=0)
        # robot.syncWrite()
        
        invPttrn["la_Ka"],invPttrn["t15"],invPttrn["t17"]=resp.lengthLA.data,t15,t17

    # rospy.loginfo("sudut: %s" % resp.angleServo.data)

    return resp.angleServo.data

def invers_walk(robot,dxl,base,x,y,z,times,condition='walk1'):

    IK_w=rospy.ServiceProxy('compute_invers_walk',ComputeInversWalk)
    req=ComputeInversWalkRequest()
    
    req.base.data=base
    req.coordinatX.data=x
    req.coordinatY.data=y
    req.coordinatZ.data=z
    if base=='ki':
        req.length_la.data=invPttrn["la_Ki"]
        req.t_EngkleBaseRoll.data=invPttrn["t18"]
        req.t_EngkleBasePitch.data=-invPttrn["t16"]
    elif base=='ka':
        req.length_la.data=invPttrn["la_Ka"]
        req.t_EngkleBaseRoll.data=invPttrn["t17"]
        req.t_EngkleBasePitch.data=invPttrn["t15"]
    
    resp=IK_w.call(req)
    angle = [float(x) for x in resp.angleServo.data.split(",")]
    t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18=angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11]

    if condition=='walk1':
        #send servo left leg
        dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
        dxl[9].moveSync(t10,times,dxl[9].prevGoal,read=0)
        dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
        dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
        dxl[15].moveSync(t16,times,dxl[15].prevGoal,read=0)
        dxl[17].moveSync(t18,times,dxl[17].prevGoal,read=0)

        #send servo right leg
        dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
        dxl[8].moveSync(t9,times,dxl[8].prevGoal,read=0)
        dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
        dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
        dxl[14].moveSync(t15,times,dxl[14].prevGoal,read=0)
        dxl[16].moveSync(t17,times,dxl[16].prevGoal,read=0)

    elif condition=='walk2':
        if base=='ki':
            dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
            dxl[9].moveSync(invPttrn["t10"],times,dxl[9].prevGoal,read=0)
            dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
            dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
            dxl[15].moveSync(invPttrn["t16"],times,dxl[15].prevGoal,read=0)
            dxl[17].moveSync(invPttrn["t18"],times,dxl[17].prevGoal,read=0)

            dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
            dxl[8].moveSync(invPttrn["t9"],times,dxl[8].prevGoal,read=0)
            dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
            dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
            dxl[14].moveSync(t15,times,dxl[14].prevGoal,read=0)
            dxl[16].moveSync(invPttrn["t17"],times,dxl[16].prevGoal,read=0)
        elif base=='ka':
            #send servo left leg
            dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
            dxl[9].moveSync(invPttrn["t10"],times,dxl[9].prevGoal,read=0)
            dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
            dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
            dxl[15].moveSync(t16,times,dxl[15].prevGoal,read=0)
            dxl[17].moveSync(invPttrn["t18"],times,dxl[17].prevGoal,read=0)

            #send servo right leg
            dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
            dxl[8].moveSync(invPttrn["t9"],times,dxl[8].prevGoal,read=0)
            dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
            dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
            dxl[14].moveSync(invPttrn["t15"],times,dxl[14].prevGoal,read=0)
            dxl[16].moveSync(invPttrn["t17"],times,dxl[16].prevGoal,read=0)

def invers_walk2(robot,dxl,base,x,y,z,times,vRoll,vPitch,condition='walk1'):

    IK_w=rospy.ServiceProxy('compute_invers_walk',ComputeInversWalk)
    req=ComputeInversWalkRequest()
    
    req.base.data=base
    req.coordinatX.data=x
    req.coordinatY.data=y
    req.coordinatZ.data=z
    if base=='ki':
        req.length_la.data=invPttrn["la_Ki"]
        req.t_EngkleBaseRoll.data=invPttrn["t18"]
        req.t_EngkleBasePitch.data=-invPttrn["t16"]
    elif base=='ka':
        req.length_la.data=invPttrn["la_Ka"]
        req.t_EngkleBaseRoll.data=invPttrn["t17"]
        req.t_EngkleBasePitch.data=invPttrn["t15"]
    
    resp=IK_w.call(req)
    angle = [float(x) for x in resp.angleServo.data.split(",")]
    t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18=angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11]

    if condition=='walk1':
        #send servo left leg
        dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
        dxl[9].moveSync(t10,times,dxl[9].prevGoal,read=0)
        dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
        dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
        dxl[15].moveSync(t16,times,dxl[15].prevGoal,read=0)
        dxl[17].moveSync(t18,times,dxl[17].prevGoal,read=0)

        #send servo right leg
        dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
        dxl[8].moveSync(t9,times,dxl[8].prevGoal,read=0)
        dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
        dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
        dxl[14].moveSync(t15,times,dxl[14].prevGoal,read=0)
        dxl[16].moveSync(t17,times,dxl[16].prevGoal,read=0)

    elif condition=='walk2':
        if base=='ki':
            dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
            dxl[9].moveSync(invPttrn["t10"],times,dxl[9].prevGoal,read=0)
            dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
            dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
            dxl[15].moveSync(invPttrn["t16"],vPitch,dxl[15].prevGoal,time_type='omega',read=0)
            dxl[17].moveSync(invPttrn["t18"],vRoll,dxl[17].prevGoal,time_type='omega',read=0)

            dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
            dxl[8].moveSync(invPttrn["t9"],times,dxl[8].prevGoal,read=0)
            dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
            dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
            dxl[14].moveSync(t15,times,dxl[14].prevGoal,read=0)
            dxl[16].moveSync(invPttrn["t17"],times,dxl[16].prevGoal,read=0)
        elif base=='ka':
            #send servo left leg
            dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
            dxl[9].moveSync(invPttrn["t10"],times,dxl[9].prevGoal,read=0)
            dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
            dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
            dxl[15].moveSync(t16,times,dxl[15].prevGoal,read=0)
            dxl[17].moveSync(invPttrn["t18"],times,dxl[17].prevGoal,read=0)

            #send servo right leg
            dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
            dxl[8].moveSync(invPttrn["t9"],times,dxl[8].prevGoal,read=0)
            dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
            dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
            dxl[14].moveSync(invPttrn["t15"],vPitch,dxl[14].prevGoal,time_type='omega',read=0)
            dxl[16].moveSync(invPttrn["t17"],vRoll,dxl[16].prevGoal,time_type='omega',read=0)
    
def invers_walkrey(robot,dxl,base,x,y,z,times,vRoll,vPitch):

    IK_w=rospy.ServiceProxy('compute_invers_walk',ComputeInversWalk)
    req=ComputeInversWalkRequest()
    
    req.base.data=base
    req.coordinatX.data=x
    req.coordinatY.data=y
    req.coordinatZ.data=z
    if base=='ki':
        req.length_la.data=invPttrn["la_Ki"]
        req.t_EngkleBaseRoll.data=invPttrn["t18"]
        req.t_EngkleBasePitch.data=-invPttrn["t16"]
    elif base=='ka':
        req.length_la.data=invPttrn["la_Ka"]
        req.t_EngkleBaseRoll.data=invPttrn["t17"]
        req.t_EngkleBasePitch.data=invPttrn["t15"]
    
    resp=IK_w.call(req)
    angle = [float(x) for x in resp.angleServo.data.split(",")]
    t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18=angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11]

    #for swing planning
    # swngPlan["xbase"],swngPlan["xswing"]=resp.xbase.data,resp.xswing.data
    # print("xbase,xswing",swngPlan["xbase"],swngPlan["xswing"])

    if base=='ki':
        # dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
        dxl[9].moveSync(invPttrn["t10"],1.5,dxl[9].prevGoal,read=0)
        dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
        dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
        dxl[15].moveSync(invPttrn["t16"],vPitch,dxl[15].prevGoal,time_type='omega',read=0)
        dxl[17].moveSync(invPttrn["t18"],vRoll,dxl[17].prevGoal,time_type='omega',read=0)

        # dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
        dxl[8].moveSync(invPttrn["t9"],times,dxl[8].prevGoal,read=0)
        dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
        dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
        dxl[14].moveSync(t15,times,dxl[14].prevGoal,read=0)
        dxl[16].moveSync(invPttrn["t17"],times,dxl[16].prevGoal,read=0)
    elif base=='ka':
        #send servo left leg
        # dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
        dxl[9].moveSync(invPttrn["t10"],times,dxl[9].prevGoal,read=0)
        dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
        dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
        dxl[15].moveSync(t16,times,dxl[15].prevGoal,read=0)
        dxl[17].moveSync(invPttrn["t18"],times,dxl[17].prevGoal,read=0)

        #send servo right leg
        # dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
        dxl[8].moveSync(invPttrn["t9"],1.5,dxl[8].prevGoal,read=0)
        dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
        dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
        dxl[14].moveSync(invPttrn["t15"],vPitch,dxl[14].prevGoal,time_type='omega',read=0)
        dxl[16].moveSync(invPttrn["t17"],vRoll,dxl[16].prevGoal,time_type='omega',read=0)
 
def invers_walk5(robot,dxl,base,x,y,z,times,vRoll,vPitch,condition='walk1'):

    IK_w=rospy.ServiceProxy('compute_invers_walk',ComputeInversWalk0)
    req=ComputeInversWalk0Request()
    
    req.base.data=base
    req.coordinatX.data=x
    req.coordinatY.data=y
    req.coordinatZ.data=z
    if base=='ki':
        req.length_la.data=invPttrn["la_Ki"]
        req.t_EngkleBaseRoll.data=invPttrn["t18"]
        req.t_EngkleBasePitch.data=-invPttrn["t16"]
    elif base=='ka':
        req.length_la.data=invPttrn["la_Ka"]
        req.t_EngkleBaseRoll.data=invPttrn["t17"]
        req.t_EngkleBasePitch.data=invPttrn["t15"]
    
    resp=IK_w.call(req)
    angle = [float(x) for x in resp.angleServo.data.split(",")]
    t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18=angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11]

    #for swing planning
    swngPlan["xbase"],swngPlan["xswing"]=resp.xbase.data,resp.xswing.data
    # print("xbase,xswing",swngPlan["xbase"],swngPlan["xswing"])

    if condition=='walk1':
        #send servo left leg
        dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
        dxl[9].moveSync(t10,times,dxl[9].prevGoal,read=0)
        dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
        dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
        dxl[15].moveSync(t16,times,dxl[15].prevGoal,read=0)
        dxl[17].moveSync(t18,times,dxl[17].prevGoal,read=0)

        #send servo right leg
        dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
        dxl[8].moveSync(t9,times,dxl[8].prevGoal,read=0)
        dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
        dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
        dxl[14].moveSync(t15,times,dxl[14].prevGoal,read=0)
        dxl[16].moveSync(t17,times,dxl[16].prevGoal,read=0)

    elif condition=='walk2':
        if base=='ki':
            dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
            dxl[9].moveSync(invPttrn["t10"],1.5,dxl[9].prevGoal,read=0)
            dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
            dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
            dxl[15].moveSync(invPttrn["t16"],vPitch,dxl[15].prevGoal,time_type='omega',read=0)
            dxl[17].moveSync(invPttrn["t18"],vRoll,dxl[17].prevGoal,time_type='omega',read=0)

            dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
            dxl[8].moveSync(invPttrn["t9"],times,dxl[8].prevGoal,read=0)
            dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
            dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
            dxl[14].moveSync(t15,times,dxl[14].prevGoal,read=0)
            dxl[16].moveSync(invPttrn["t17"],times,dxl[16].prevGoal,read=0)
        elif base=='ka':
            #send servo left leg
            dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
            dxl[9].moveSync(invPttrn["t10"],times,dxl[9].prevGoal,read=0)
            dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
            dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
            dxl[15].moveSync(t16,times,dxl[15].prevGoal,read=0)
            dxl[17].moveSync(invPttrn["t18"],times,dxl[17].prevGoal,read=0)

            #send servo right leg
            dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
            dxl[8].moveSync(invPttrn["t9"],1.5,dxl[8].prevGoal,read=0)
            dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
            dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
            dxl[14].moveSync(invPttrn["t15"],vPitch,dxl[14].prevGoal,time_type='omega',read=0)
            dxl[16].moveSync(invPttrn["t17"],vRoll,dxl[16].prevGoal,time_type='omega',read=0)    

def forward(robot,dxl,base,nFrame=27,readAll_leg='yes'):
    if readAll_leg=='yes':
        angle=robot.readLeg()
        angleLeg="%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i" % (angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11])

    elif readAll_leg=='virtual':
        t7=dxl[6].prevGoalDegree
        t8=dxl[7].prevGoalDegree
        t9=dxl[8].prevGoalDegree 
        t10=dxl[9].prevGoalDegree 
        t11=dxl[10].prevGoalDegree
        t12=dxl[11].prevGoalDegree
        t13=dxl[12].prevGoalDegree
        t14=dxl[13].prevGoalDegree
        t15=dxl[14].prevGoalDegree
        t16=dxl[15].prevGoalDegree
        t17=dxl[16].prevGoalDegree
        t18=dxl[17].prevGoalDegree
        angleLeg="%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i" % (t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18)
    
    forward_leg=rospy.ServiceProxy('compute_forward_kinematics',ComputeForwardKinematics)
    req=ComputeForwardKinematicsRequest()
    req.base.data=base
    req.angle.data=angleLeg
    req.nFrame.data=nFrame
    
    resp=forward_leg.call(req)
    # rospy.loginfo("X: %s" % resp.resultX.data)
    # rospy.loginfo("Y: %s" % resp.resultY.data)
    # rospy.loginfo("Z: %s" % resp.resultZ.data)
    fwdNow["x"],fwdNow["y"],fwdNow["z"]=round(resp.resultX.data,3),round(resp.resultY.data,3),round(resp.resultZ.data,3)

    return round(resp.resultX.data,3),round(resp.resultY.data,3),round(resp.resultZ.data,3)

def COM(robot,dxl,base,readAll_leg='yes',read_type='native',nFwd=26):
    # print("nfwd=",nFwd)
    if readAll_leg=='base': 
        if read_type=='native':
            if base=='ki':
                t7=dxl[6].prevGoalDegree
                t8=dxl[7].prevGoalDegree
                t9=dxl[8].prevGoalDegree
                t10=robot.readOne(10)
                t11=dxl[10].prevGoalDegree
                t12=robot.readOne(12)
                t13=dxl[12].prevGoalDegree
                t14=dxl[13].prevGoalDegree
                t15=dxl[14].prevGoalDegree 
                t16=robot.readOne(16)
                t17=dxl[16].prevGoalDegree 
                t18=robot.readOne(18)
            elif base=='ka':
                t7=dxl[6].prevGoalDegree
                t8=dxl[7].prevGoalDegree
                t9=robot.readOne(9)
                t10=dxl[9].prevGoalDegree
                t11=robot.readOne(11)
                t12=dxl[11].prevGoalDegree
                t13=dxl[12].prevGoalDegree
                t14=dxl[13].prevGoalDegree
                t15=robot.readOne(15)
                t16=dxl[15].prevGoalDegree 
                t17=robot.readOne(17)
                t18=dxl[17].prevGoalDegree
        elif read_type=='ros':
            if base=='ki':
                t7=dxl[6].prevGoalDegree
                t8=dxl[7].prevGoalDegree
                t9=dxl[8].prevGoalDegree
                t10=robot.rosReadOne(10)
                t11=dxl[10].prevGoalDegree
                t12=robot.rosReadOne(12)
                t13=dxl[12].prevGoalDegree
                t14=dxl[13].prevGoalDegree
                t15=dxl[14].prevGoalDegree 
                t16=robot.rosReadOne(16)
                t17=dxl[16].prevGoalDegree 
                t18=robot.rosReadOne(18)
            elif base=='ka':
                t7=dxl[6].prevGoalDegree
                t8=dxl[7].prevGoalDegree
                t9=robot.rosReadOne(9)
                t10=dxl[9].prevGoalDegree
                t11=robot.rosReadOne(11)
                t12=dxl[11].prevGoalDegree
                t13=dxl[12].prevGoalDegree
                t14=dxl[13].prevGoalDegree
                t15=robot.rosReadOne(15)
                t16=dxl[15].prevGoalDegree 
                t17=robot.rosReadOne(17)
                t18=dxl[17].prevGoalDegree

        angleLeg="%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i" % (t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18) 

    elif readAll_leg=='yes':
        if read_type=='native':
            angle=robot.readLeg()
        elif read_type=='ros':
            angle=robot.rosReadLeg()
        angleLeg="%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i" % (angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11])
        # print(angleLeg)

    elif readAll_leg=='virtual':
        t7=dxl[6].prevGoalDegree
        t8=dxl[7].prevGoalDegree
        t9=dxl[8].prevGoalDegree 
        t10=dxl[9].prevGoalDegree 
        t11=dxl[10].prevGoalDegree
        t12=dxl[11].prevGoalDegree
        t13=dxl[12].prevGoalDegree
        t14=dxl[13].prevGoalDegree
        t15=dxl[14].prevGoalDegree
        t16=dxl[15].prevGoalDegree
        t17=dxl[16].prevGoalDegree
        t18=dxl[17].prevGoalDegree
        angleLeg="%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i" % (t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18)

    # print(angleLeg)
    com=rospy.ServiceProxy('compute_COM',ComputeCOM)
    req=ComputeCOMRequest()
    req.base.data=str(base)
    req.angle.data=str(angleLeg)
    req.nFwd.data=nFwd
    resp=com.call(req)

    fwd=[float(x) for x in resp.resp_forward.data.split(",")]
    fwdNow["x"],fwdNow["y"],fwdNow["z"]=round(fwd[0],3),round(fwd[1],3),round(fwd[2],3)
    comNow["x"],comNow["y"],comNow["z"]=round(resp.comX.data,3),round(resp.comY.data,3),round(resp.comZ.data,3)

    print("comX",round(resp.comX.data,3))
    print("comY",round(resp.comY.data,3))
    print("comZ",round(resp.comZ.data,3))
    print("fwd:",resp.resp_forward.data)

    return round(resp.comX.data,3),round(resp.comY.data,3),round(resp.comZ.data,3)

#-----------------------------------------------------------------------------------------

def walkUpdate(robot,dxl,t,tsup,base,xGoal,firstStep,lastStep,condition='normal'):
    t=t/1000000 # ubah t dari microsecond ke second
    
    if base==-1: #base kaki kiri
        yg=-1 #pengali untuk merubah kaki tumpuan
        if condition=='normal':
            COM(robot,dxl,'ki',readAll_leg='base')
        elif condition=='virtual':
            COM(robot,dxl,'ki',readAll_leg='virtual')
    elif base==1: #base kaki kanan
        yg=1
        if condition=='normal':
            COM(robot,dxl,'ka',readAll_leg='base')
        elif condition=='virtual':
            COM(robot,dxl,'ka',readAll_leg='virtual')

    print("t:",t)

    #periode 1
    if t<=(tsup/4)+0.1:
        if firstStep==1: #jika awal melangkah
            t1=0
            t2=tsup/4+0.1
            y1=-yg*comDef["y"]
            y2=yg*0
            Xt=comDef["x"]
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
            sfx=0
            sfy=0
            sfz=0
            Zt=comDef["z"]+(t*(17-comDef["z"])/(tsup/4)+0.1)
            
            pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],pttrn["xS"]=Xt,Yt,sfx,sfy,sfz,sfx
        
        elif firstStep==0: #jika bukan awal melangkah

            t1=0
            t2=tsup/4+0.1
            # x1=comXPolaPeriod[4]*(-1) #-1 karena ganti kaki tumpuan
            x1=comNow["x"]
            x2=comXPolaPeriod[4]*(-1) #-1 karena ganti kaki tumpuan
            # x2=comDef["x"]
            y1=comYPolaPeriod[4]*(-1) #-1 karena ganti kaki tumpuan
            y2=yg*0

            sfx=(fwdNow["x"]/10)-((t*comNow["x"]/10)/((tsup/4)+0.1))
            print("fwdXNow:",fwdNow["x"])
            sfy=0
            sfz=sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
            if sfz<0:
                sfz=0

            Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
           
            pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],pttrn["xS"]=Xt,Yt,sfx,sfy,sfz,sfx
         
        comXPolaPeriod[1]=Xt
        comYPolaPeriod[1]=Yt
        print("==================Periode 1=====================")
        
    #periode 2
    elif (t>(tsup/4)+0.1) and (t<=(tsup/2)+0.1):
        
        t1=(tsup/4)+0.1
        t2=(tsup/2)+0.1
        y1=comYPolaPeriod[1]
        y2=yg*0  #yg*0
        # x1=comXPolaPeriod[1]
        # x2=(xGoal*10/2)+comDef["x"]
        # Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        Xt=comDef["x"]
        # if firstStep==1:
        #     Xt=comDef["x"]
        # elif firstStep==0:
        #     x1=comXPolaPeriod[1]
        #     x2=(xGoal*10/2)+comDef["x"]
        #     Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)

        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
        comXPolaPeriod[2]=Xt
        comYPolaPeriod[2]=Yt

        sfy=0
        sfx= (((xGoal-pttrn["xS"])/(2*3.14))*(((2*3.14*(t-(tsup/4)))/(tsup/2))-sin((2*3.14*(t-(tsup/4)))/(tsup/2))))+pttrn["xS"]
        sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))

        pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"]=Xt,Yt,sfx,sfy,sfz

        print("==================Periode 2=====================")
    
    #periode 3
    elif (t>(tsup/2)+0.1) and (t<=(3*tsup/4)+0.15):
        t1=(tsup/2)+0.1
        t2=(3*tsup/4)+0.15
        x1=comXPolaPeriod[2]      
        x2=(xGoal*10/2)+comDef["x"]
        y1=comYPolaPeriod[2]
        y2=yg*0 #yg*0 15
        Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)

        sfy=0
        sfx= (((xGoal-pttrn["xS"])/(2*3.14))*(((2*3.14*(t-(tsup/4)))/(tsup/2))-sin((2*3.14*(t-(tsup/4)))/(tsup/2))))+pttrn["xS"]
        sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
        if sfz<0:
            sfz=0

        if sfx>xGoal:
            sfx=xGoal

        comXPolaPeriod[3]=Xt
        comYPolaPeriod[3]=Yt
        
        pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"]=Xt,Yt,sfx,sfy,sfz
        print("==================Periode 3=====================")

    # #periode 4
    elif (t>(3*tsup/4)+0.15) and (t<=tsup+0.25):
        
        #jika bukan akhir langkah (akan melanjutkan berjalan kembali)
        if lastStep==0:
            
            t1=(3*tsup/4)+0.15
            t2=(tsup)+0.25
            x1=comXPolaPeriod[3]
            x2=(xGoal*10/2)+comDef["x"]
            y1=comYPolaPeriod[3]
            y2=-yg*comDef["y"]
            Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
            sfx=xGoal+((t-(3*tsup/4))*xGoal/(tsup/4))
            sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
            if sfz<0:
                sfz=0

            if sfx>xGoal:
                sfx=xGoal
            
            sfy=0

        # jika akhir langkah dan akan berhenti (kaki sejajar)
        elif lastStep==1:
            
            t1=(3*tsup/4)+0.15
            t2=(tsup)+0.25
            x1=comXPolaPeriod[3]
            x2=comDef["x"]
            y1=comYPolaPeriod[3]
            y2=-yg*comDef["y"]
            Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
            sfx=xGoal+((t-(3*tsup/4))*xGoal/(tsup/4))
            sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
            if sfz<0:
                sfz=0

            if sfx>xGoal:
                sfx=xGoal
            sfy=0
        
        comXPolaPeriod[4]=Xt
        comYPolaPeriod[4]=Yt
        pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],pttrn["xS"]=Xt,Yt,sfx,sfy,sfz,sfx
        print("==================Periode 4=====================")

    print("base kaki kiri" if base==-1 else "base kaki kanan" )
    print("Xt=",Xt)
    print("Yt=",Yt)
    print("swing kaki kanan" if base==-1 else "swing kaki kiri" )
    print("sfx=",sfx)
    print("sfy=",sfy)
    print("sfz=",sfz)
    print("================================================")

    # return Xt,Yt,sfx,sfy,sfz

def walkUpdate2(robot,dxl,t,tsup,base,xGoal,firstStep,lastStep,condition='normal'):
    t=t/1000000 # ubah t dari microsecond ke second
    print("first step=",firstStep)
    if base==-1: #base kaki kiri
        yg=-1 #pengali untuk merubah kaki tumpuan
        Xtc=1
        Ytc=4.8
        if condition=='normal':
            COM(robot,dxl,'ki',readAll_leg='base')
        elif condition=='virtual':
            COM(robot,dxl,'ki',readAll_leg='virtual')
    elif base==1: #base kaki kanan
        yg=1
        Ytc=4.5
        Xtc=1
        if condition=='normal':
            COM(robot,dxl,'ka',readAll_leg='base')
        elif condition=='virtual':
            COM(robot,dxl,'ka',readAll_leg='virtual')

    print("t:",t)

    #periode 1
    if t<=(tsup/4)+0.1:
        if firstStep==1: #jika awal melangkah
            t1=0
            t2=tsup/4+0.1
            y1=-yg*comDef["y"]
            y2=yg*0
            Xt=comDef["x"]
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
            sfx=0
            sfy=0
            sfz=0
            
            # Xt=comDef["x"]-(t*comDef["x"]/((tsup/4)+0.1))
            # Yt=((t*yg*Ytc)/((tsup/4)+0.1))
            # sfx=(fwdNow["x"]/10)-((t*comNow["x"]/10)/((tsup/4)+0.1))
            # sfy=0
            # sfz=0
            # Zt=comDef["z"]+(t*(17-comDef["z"])/(tsup/4)+0.1)
            
            pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],pttrn["xS"]=Xt,Yt,sfx,sfy,sfz,sfx
        
        elif firstStep==0: #jika bukan awal melangkahs0000                                                                                                         0
            t1=0
            t2=tsup/4+0.1
            x1=comNow["x"]
            x2=comXPolaPeriod[4]*(-1) #-1 karena ganti kaki tumpuan
            y1=comYPolaPeriod[4]*(-1) #-1 karena ganti kaki tumpuan
            y2=yg*0

            sfx=(fwdNow["x"]/10)-((t*comNow["x"]/10)/((tsup/4)+0.1))
            print("fwdXNow:",fwdNow["x"])
            sfy=0
            sfz=sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
            if sfz<0:
                sfz=0

            Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
           
            pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],pttrn["xS"]=Xt,Yt,sfx,sfy,sfz,sfx
         
        comXPolaPeriod[1]=Xt
        comYPolaPeriod[1]=Yt
        print("==================Periode 1=====================")
        

    #periode 2
    elif (t>(tsup/4)+0.1) and (t<=(tsup/2)+0.1):
        Xt=0
        Yt=yg*Ytc
        comXPolaPeriod[2]=Xt
        comYPolaPeriod[2]=Yt

        sfy=0
        sfx= (((xGoal-pttrn["xS"])/(2*3.14))*(((2*3.14*(t-(tsup/4)))/(tsup/2))-sin((2*3.14*(t-(tsup/4)))/(tsup/2))))+pttrn["xS"]
        sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))

        pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"]=Xt,Yt,sfx,sfy,sfz

        print("==================Periode 2=====================")
    
    #periode 3
    elif (t>(tsup/2)+0.1) and (t<=(3*tsup/4)+0.15):
        t1=(tsup/2)+0.1
        t2=(3*tsup/4)+0.15
        x1=comXPolaPeriod[2]      
        x2=(xGoal*10/2)+comDef["x"]
        y1=comYPolaPeriod[2]
        y2=yg*0 #yg*0 15
        Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)

        # Xt=((t-(tsup/2))*1/(tsup/4))
        # Yt=(yg*Ytc)-((t-(tsup/2))*(1*yg)/(tsup/4))-3

        sfy=0
        sfx= (((xGoal-pttrn["xS"])/(2*3.14))*(((2*3.14*(t-(tsup/4)))/(tsup/2))-sin((2*3.14*(t-(tsup/4)))/(tsup/2))))+pttrn["xS"]
        sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
        if sfz<0:
            sfz=0

        if sfx>xGoal:
            sfx=xGoal

        comXPolaPeriod[3]=Xt
        comYPolaPeriod[3]=Yt
        
        pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"]=Xt,Yt,sfx,sfy,sfz
        print("==================Periode 3=====================")

    #periode 4
    elif (t>(3*tsup/4)+0.15) and (t<=tsup+0.1):
        t1=(3*tsup/4)+0.15
        t2=(tsup)+0.1
        x1=comXPolaPeriod[3]
        x2=(xGoal*10/2)+comDef["x"]+15
        y1=comYPolaPeriod[3]
        y2=(-yg*comDef["y"])-18
        Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
        sfx=xGoal+((t-(3*tsup/4))*xGoal/(tsup/4))
        sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
        if sfz<0:
            sfz=0
        if sfx>xGoal:
            sfx=xGoal
        sfy=0

        # Xt=((t-(3*tsup/4))*xGoal/(tsup/4))
        # Yt=(4*yg)-((t-(3*tsup/4))*(4*yg)/(tsup/4))
        # sfx=xGoal+((t-(3*tsup/4))*xGoal/(tsup/4))
        # sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
        # if sfz<0:
        #     sfz=0

        # if sfx>xGoal:
        #     sfx=xGoal
            
        # sfy=0

        comXPolaPeriod[4]=Xt
        comYPolaPeriod[4]=Yt
        pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],pttrn["xS"]=Xt,Yt,sfx,sfy,sfz,sfx
        print("==================Periode 4=====================")

    print("base kaki kiri" if base==-1 else "base kaki kanan" )
    print("Xt=",Xt)
    print("Yt=",Yt)
    print("swing kaki kanan" if base==-1 else "swing kaki kiri" )
    print("sfx=",sfx)
    print("sfy=",sfy)
    print("sfz=",sfz)
    print("================================================")

    # return Xt,Yt,sfx,sfy,sfz

def walkUpdateove(robot,dxl,t,tsup,base,xGoal,firstStep,lastStep,condition='normal'):
    t=t/1000000 # ubah t dari microsecond ke second
    
    if base==-1: #base kaki kiri
        yg=-1 #pengali untuk merubah kaki tumpuan
        if condition=='normal':
            COM(robot,dxl,'ki',readAll_leg='base')
        elif condition=='virtual':
            COM(robot,dxl,'ki',readAll_leg='virtual')
    elif base==1: #base kaki kanan
        yg=1
        if condition=='normal':
            COM(robot,dxl,'ka',readAll_leg='base')
        elif condition=='virtual':
            COM(robot,dxl,'ka',readAll_leg='virtual')

    print("t:",t)
    sH=2
    #periode 1
    if t<=(tsup/4)+0.1:
        if firstStep==1: #jika awal melangkah
            t1=0
            t2=tsup/4+0.1
            y1=-yg*comDef["y"]
            y2=yg*0
            Xt=comDef["x"]
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
            sfx=0
            sfy=0
            sfz=0
            Zt=comDef["z"]+(t*(17-comDef["z"])/(tsup/4)+0.1)
            
            pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],pttrn["xS"]=Xt,Yt,sfx,sfy,sfz,sfx
        
        elif firstStep==0: #jika bukan awal melangkah
            t1=0
            t2=tsup/4+0.1
            x1=comNow["x"]
            x2=comXPolaPeriod[4]*(-1)+10
            y1=comYPolaPeriod[4]*(-1)
            y2=yg*0

            sfx=(fwdNow["x"]/10)-((t*comNow["x"]/10)/((tsup/4)+0.1))
            print("fwdXNow:",fwdNow["x"])
            sfy=0
            sfz=sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
            if sfz<0:
                sfz=0

            Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
           
            pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],pttrn["xS"]=Xt,Yt,sfx,sfy,sfz,sfx
         
        comXPolaPeriod[1]=Xt
        comYPolaPeriod[1]=Yt
        print("==================Periode 1=====================")
        
    #periode 2
    elif (t>(tsup/4)+0.1) and (t<=(tsup/2)+0.1):
        
        t1=(tsup/4)+0.1
        t2=(tsup/2)+0.1
        y1=comYPolaPeriod[1]
        y2=yg*0 
        x1=comXPolaPeriod[1]
        x2=(xGoal*10/2)+comDef["x"]
        Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)

        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
        comXPolaPeriod[2]=Xt
        comYPolaPeriod[2]=Yt

        sfy=0
        sfx= (((xGoal-pttrn["xS"])/(2*3.14))*(((2*3.14*(t-(tsup/4)))/(tsup/2))-sin((2*3.14*(t-(tsup/4)))/(tsup/2))))+pttrn["xS"]
        sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))

        pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"]=Xt,Yt,sfx,sfy,sfz

        print("==================Periode 2=====================")
    
    #periode 3
    elif (t>(tsup/2)+0.1) and (t<=(3*tsup/4)+0.15):
        t1=(tsup/2)+0.1
        t2=(3*tsup/4)+0.15
        x1=comXPolaPeriod[2]      
        x2=(xGoal*10/2)+comDef["x"]
        y1=comYPolaPeriod[2]
        y2=yg*0 #yg*0 15
        Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)

        sfy=0
        sfx= (((xGoal-pttrn["xS"])/(2*3.14))*(((2*3.14*(t-(tsup/4)))/(tsup/2))-sin((2*3.14*(t-(tsup/4)))/(tsup/2))))+pttrn["xS"]
        sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
        if sfz<0:
            sfz=0

        if sfx>xGoal:
            sfx=xGoal

        comXPolaPeriod[3]=Xt
        comYPolaPeriod[3]=Yt
        
        pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"]=Xt,Yt,sfx,sfy,sfz
        print("==================Periode 3=====================")

    # #periode 4
    elif (t>(3*tsup/4)+0.15) and (t<=tsup+0.1):
        t1=(3*tsup/4)+0.15
        t2=(tsup)+0.1
        x1=comXPolaPeriod[3]
        x2=(xGoal*10/2)+comDef["x"]+20
        y1=comYPolaPeriod[3]
        y2=(-yg*comDef["y"])
        Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
        sfx=xGoal+((t-(3*tsup/4))*xGoal/(tsup/4))
        sfz = sH-((sH)*(1-sin((3.14*((t-(tsup/4))))/(tsup/2))))
        if sfz<0:
            sfz=0
        if sfx>xGoal:
            sfx=xGoal
        sfy=0

        comXPolaPeriod[4]=Xt
        comYPolaPeriod[4]=Yt
        pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],pttrn["xS"]=Xt,Yt,sfx,sfy,sfz,sfx
        print("==================Periode 4=====================")    
        
    print("base kaki kiri" if base==-1 else "base kaki kanan" )
    print("Xt=",Xt)
    print("Yt=",Yt)
    print("swing kaki kanan" if base==-1 else "swing kaki kiri" )
    print("sfx=",sfx)
    print("sfy=",sfy)
    print("sfz=",sfz)
    print("================================================")

    # return Xt,Yt,sfx,sfy,sfz

def walkUpdateove2(robot,dxl,t,tsup,base,xGoal,firstStep,lastStep,condition='normal'):
    t=t/1000000 # ubah t dari microsecond ke second
    
    if base==-1: #base kaki kiri
        yg=-1 #pengali untuk merubah kaki tumpuan
        if condition=='normal':
            COM(robot,dxl,'ki',readAll_leg='base')
        elif condition=='virtual':
            COM(robot,dxl,'ki',readAll_leg='virtual')
    elif base==1: #base kaki kanan
        yg=1
        if condition=='normal':
            COM(robot,dxl,'ka',readAll_leg='base')
        elif condition=='virtual':
            COM(robot,dxl,'ka',readAll_leg='virtual')

    pttrn["Zt"]=comNow["z"]
    print("t:",t)

    #periode 1
    if t<=(tsup/4)+0.1:
        if firstStep==1: #jika awal melangkah
            t1=0
            t2=tsup/4+0.1
            y1=-yg*comDef["y"]
            y2=yg*0
            Xt=comDef["x"]
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
            # Zt=comDef["z"]+(t*(17-comDef["z"])/(tsup/4)+0.1)
        
        elif firstStep==0: #jika bukan awal melangkah
            t1=0
            t2=tsup/4+0.1
            x1=comNow["x"]
            x2=comXPolaPeriod[4]*(-1)+10
            y1=comYPolaPeriod[4]*(-1)
            y2=yg*0

            Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
         
        comXPolaPeriod[1]=Xt
        comYPolaPeriod[1]=Yt
        print("==================Periode 1=====================")
        
    #periode 2
    elif (t>(tsup/4)+0.1) and (t<=(tsup/2)+0.1):
        
        t1=(tsup/4)+0.1
        t2=(tsup/2)+0.1
        y1=comYPolaPeriod[1]
        y2=yg*0 
        x1=comXPolaPeriod[1]
        x2=(xGoal*10/2)+comDef["x"]
        Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)

        comXPolaPeriod[2]=Xt
        comYPolaPeriod[2]=Yt
        print("==================Periode 2=====================")
    
    #periode 3
    elif (t>(tsup/2)+0.1) and (t<=(3*tsup/4)+0.15):
        t1=(tsup/2)+0.1
        t2=(3*tsup/4)+0.15
        x1=comXPolaPeriod[2]      
        x2=(xGoal*10/2)+comDef["x"]
        y1=comYPolaPeriod[2]
        y2=yg*0 #yg*0 15
        Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)

        comXPolaPeriod[3]=Xt
        comYPolaPeriod[3]=Yt
        print("==================Periode 3=====================")

    # #periode 4
    elif (t>(3*tsup/4)+0.15) and (t<=tsup+0.1):
        t1=(3*tsup/4)+0.15
        t2=(tsup)+0.1
        x1=comXPolaPeriod[3]
        x2=(xGoal*10/2)+comDef["x"]+20
        y1=comYPolaPeriod[3]
        y2=(-yg*comDef["y"])
        Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)

        comXPolaPeriod[4]=Xt
        comYPolaPeriod[4]=Yt
        print("==================Periode 4=====================")    
        

    # return Xt,Yt,sfx,sfy,sfz
    #-----------pattern swing------------------------
    sH=2.5#tinggi maksimum langkah

    if t<=(tsup/4):
        sfx=(fwdDef["x"]/10)
        sfy=0
        sfz=0

    elif t>(tsup/4):
        t1=tsup/4
        t2=tsup

        sfx1=fwdDef["x"]/10 #posisi kaki awal sumbu x (cm)
        sfx2=xGoal #posisi kaki tujuan

        if t<=(5/8)*tsup: #3/2 dibagi 2, biar lebih mulus
            tz1=tsup/4
            tz2=((5/8)*tsup)
            sfz1=pttrn["sfz"]
            sfz2=sH
        elif t>(5/8)*tsup:
            tz1=((5/8)*tsup)
            tz2=tsup+0.25
            sfz1=pttrn["sfz"]
            sfz2=0

        sfy=0
        sfx=round(((t-t1)/(t2-t1)*(sfx2-sfx1))+sfx1,3)
        sfz=round(((t-tz1)/(tz2-tz1)*(sfz2-sfz1))+sfz1,3)
        if sfz<0:
            sfz=0

    pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"]=Xt,Yt,sfx,sfy,sfz

    print("base kaki kiri" if base==-1 else "base kaki kanan" )
    print("Xt=",Xt)
    print("Yt=",Yt)
    print("swing kaki kanan" if base==-1 else "swing kaki kiri" )
    print("sfx=",sfx)
    print("sfy=",sfy)
    print("sfz=",sfz)
    print("================================================")

    # return Xt,Yt,sfx,sfy,sfz

def walkUpdaterey(robot,dxl,t,tsup,base,xGoal,firstStep,lastStep,condition='normal'):
    
    t=t/1000000 # ubah t dari microsecond ke second
    
    if base==-1: #base kaki kiri
        yg=-1 #pengali untuk merubah kaki tumpuan
        if condition=='normal':
            COM(robot,dxl,'ki',readAll_leg='base')
        elif condition=='virtual':
            COM(robot,dxl,'ki',readAll_leg='virtual')
    elif base==1: #base kaki kanan
        yg=1
        if condition=='normal':
            COM(robot,dxl,'ka',readAll_leg='base')
        elif condition=='virtual':
            COM(robot,dxl,'ka',readAll_leg='virtual')

    pttrn["Zt"]=comNow["z"]

    print("t:",t)

    #periode 1
    if t<=(tsup/4)+0.1:
        if firstStep==1: #jika awal melangkah
            t1=0
            t2=tsup/4+0.1
            y1=-yg*comDef["y"]
            y2=yg*0 #
            Xt=comDef["x"]
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
            
        elif firstStep==0: #jika bukan awal melangkah
          
            t1=0
            t2=tsup/4+0.1
            x1=pttrn["comXinit"]
            # x2=comDef["x"]
            
            y1=comNow["y"] #-1 karena ganti kaki tumpuan
            y2=yg*(0)
            if base==-1:
                y2=-1
                x2=comDef["x"]
            elif base==1:
                y2=0
                x2=comDef["x"]

            Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
         
        comXPolaPeriod[1]=Xt
        comYPolaPeriod[1]=Yt
        print("==================Periode 1=====================")
        
    #periode 2
    elif (t>(tsup/4)+0.1) and (t<=(tsup/2)+0.1):
        
        t1=(tsup/4)+0.1
        t2=(tsup/2)+0.1
        y1=comYPolaPeriod[1]
        if firstStep==1:
            y2=yg*0-2 #
            # Xt=comDef["x"]
            x1=comXPolaPeriod[1]
            # x2=comDef["x"]
            x2=(comDef["x"]+((xGoal*10/2)+comDef["x"]))/2
            Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        else:
            x1=comXPolaPeriod[1]
            # x2=comDef["x"]
            # Xt=comDef["x"]
            
            if base==-1:
                y2=-1
                x2=comDef["x"]
            elif base==1:
                y2=0
                x2=comDef["x"]
            
            Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)

        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
        comXPolaPeriod[2]=Xt
        comYPolaPeriod[2]=Yt

        print("==================Periode 2=====================")
    
    #periode 3
    elif (t>(tsup/2)+0.1) and (t<=(3*tsup/4)+0.15):
        t1=(tsup/2)+0.1
        t2=(3*tsup/4)+0.15

        x1=comXPolaPeriod[2]
        y1=comYPolaPeriod[2]
        if firstStep==1:
            x2=((comDef["x"]+((xGoal*10/2)+comDef["x"]))/2)
            # x2=(xGoal*10/2)+(comDef["x"])
            y2=yg*0 #
        else:
            x2=((comDef["x"]+((xGoal*10/2)+comDef["x"]))/2)
            
            if base==-1:
                y2=-1
            elif base==1:
                y2=0
        Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
        Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)

        comXPolaPeriod[3]=Xt
        comYPolaPeriod[3]=Yt
        
        print("==================Periode 3=====================")

    # #periode 4
    elif (t>(3*tsup/4)+0.15) and (t<=tsup+0.2):
        
        #jika bukan akhir langkah (akan melanjutkan berjalan kembali)
        if lastStep==0:
            
            t1=(3*tsup/4)+0.15
            t2=(tsup)+0.2
            x1=comXPolaPeriod[3]
            
            y1=comYPolaPeriod[3]

            if firstStep==1:
                if base==-1:
                    y2=-yg*comDef["y"]
                    x2=(xGoal*10/2)+(comDef["x"])+5
                elif base==1:
                    y2=-yg*comDef["y"]-5
                    x2=(xGoal*10/2)+(comDef["x"])+5
            
            elif firstStep==0:
                if base==-1:
                    y2=-yg*comDef["y"]
                    x2=(xGoal*10/2)+(comDef["x"])+5
                elif base==1:
                    y2=-yg*comDef["y"]-5
                    x2=(xGoal*10/2)+(comDef["x"])+5
            
            # y2=-yg*comDef["y"]
            Xt=round(((t-t1)/(t2-t1)*(x2-x1))+x1,3)
            Yt=round(((t-t1)/(t2-t1)*(y2-y1))+y1,3)
            
        comXPolaPeriod[4]=Xt
        comYPolaPeriod[4]=Yt
        print("==================Periode 4=====================")


    #-----------pattern swing------------------------
    sH=2#tinggi maksimum langkah

    if t<=(tsup/4):
        sfx=(fwdDef["x"]/10)
        sfy=0
        sfz=0

    elif t>(tsup/4):
        t1=tsup/4
        t2=tsup

        sfx1=fwdDef["x"]/10 #posisi kaki awal sumbu x (cm)
        sfx2=xGoal #posisi kaki tujuan

        if t<=(5/8)*tsup: #3/2 dibagi 2, biar lebih mulus
            tz1=tsup/4
            tz2=((5/8)*tsup)
            sfz1=pttrn["sfz"]
            sfz2=sH
        elif t>(5/8)*tsup:
            tz1=((5/8)*tsup)
            tz2=tsup+0.25
            sfz1=pttrn["sfz"]
            sfz2=0

        sfy=0
        sfx=round(((t-t1)/(t2-t1)*(sfx2-sfx1))+sfx1,3)
        sfz=round(((t-tz1)/(tz2-tz1)*(sfz2-sfz1))+sfz1,3)
        if sfz<0:
            sfz=0
    #---------------------------------------------------

    pttrn["Xt"],pttrn["Yt"],pttrn["sfx"],pttrn["sfy"],pttrn["sfz"]=Xt,Yt,sfx,sfy,sfz

    #for swing planning
    # swngPlan["sfx"]=pttrn["sfx"]
    
    print("base kaki kiri" if base==-1 else "base kaki kanan" )
    print("Xt=",Xt)
    print("Yt=",Yt)
    print("swing kaki kanan" if base==-1 else "swing kaki kiri" )
    print("sfx=",sfx)
    print("sfy=",sfy)
    print("sfz=",sfz)
    print("================================================")

    # return Xt,Yt,sfx,sfy,sfz

def Control(robot,dxl,base,t,condition='normal'):
    t=t/1000000 # ubah t dari microsecond ke second
    tServo=0.1

    #status com
    state1Pitch=arctan(comNow["x"]/comNow["z"])*180/pi
    state1Roll=arctan(comNow["y"]/comNow["z"])*180/pi
    # print("state1Pitch:",state1Pitch)
    # print("state1Roll:",state1Roll)

    #referensi
    refPitch=arctan(pttrn["Xt"]/comNow["z"])*180/pi
    refRoll=arctan(pttrn["Yt"]/comNow["z"])*180/pi
    # print("refPitch:",refPitch)
    # print("refRoll:",refRoll)

    angleRoll=state1Roll-refRoll
    anglePitch=state1Pitch-refPitch
    print("angleRoll:",angleRoll)
    print("anglePitch:",anglePitch)

    if base==-1: #jika tumpuan kaki kiri

        #left leg (support)
        invPttrn["t18"]=dxl[17].prevGoalDegree+angleRoll #base roll
        invPttrn["t16"]=dxl[15].prevGoalDegree+anglePitch #base pitch
        invPttrn["t10"]=0                                 #hip roll
        #right leg (swing)
        invPttrn["t17"]=dxl[16].prevGoalDegree+angleRoll #base roll
        invPttrn["t9"]=dxl[8].prevGoalDegree+angleRoll #hip roll
        # print("t9 kirim:",invPttrn["t9"])

        invers_walk(robot,dxl,'ki',pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],0.1,condition='walk2')
    
    #------------------------------------------------------------------------------------------
    elif base==1: #jika tumpuan kaki kanan

        #right leg(support) 
        invPttrn["t17"]=dxl[16].prevGoalDegree+angleRoll #base roll
        invPttrn["t15"]=dxl[14].prevGoalDegree-anglePitch #base pitch
        invPttrn["t9"]=0                                   #hip roll
        #left leg (swing)
        invPttrn["t18"]=dxl[17].prevGoalDegree+angleRoll #base roll
        invPttrn["t10"]=dxl[9].prevGoalDegree+angleRoll #hip roll

        # print("t10 kirim:",invPttrn["t10"])
        invers_walk(robot,dxl,'ka',pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],0.1,condition='walk2')
           
    if condition=='normal':
        robot.syncWrite() 
    elif condition=='virtual':
        print("sync virtual")

#-----------------------------------------------------------------------------------------
def Controlove(robot,dxl,base,t,K,condition='normal'):
    t=t/1000000 # ubah t dari microsecond ke second
    tServo=0.1
    tSmpl=0.1

    m=1.725
    g=9.80665
    l=0.19614907857545497
    ixx=0.092026292
    iyy=0.087070843
    bstate1=0.0
    bstate2=0.0

    # print("comZ:",comNow["z"])

    # #array K full state feedback
    # K=array([[8,1,0,0], #roll
    #         [0,0,15,1]]) #pitch

    #ubah status com ke sudut dan kecepatan sudut
    state1Roll=arctan(comNow["y"]/comNow["z"]) #sudut
    state2Roll=(state1Roll-controlDict["rollBef"])/tSmpl #kecepatan sudut

    state1Pitch=arctan(comNow["x"]/comNow["z"]) #sudut
    state2Pitch=(state1Pitch-controlDict["pitchBef"])/tSmpl #kecepatan sudut

    # print("state1Roll:",state1Roll)
    # print("state1Pitch:",state1Pitch)
    # print("state2Pitch:",state2Pitch)

    # controlDict["pitchBef"],controlDict["rollBef"]=state1Pitch,state1Roll

    #referensi
    refPitch=arctan(pttrn["Xt"]/comNow["z"])
    refRoll=arctan(pttrn["Yt"]/comNow["z"])
    # print("refRoll:",refRoll)
    # print("refPitch:",refPitch)

    #control parameter
    uRoll=(K[0,0]*(state1Roll-refRoll))+((K[0,1]*state2Roll))
    ARoll=degrees((-uRoll+(m*g*l*sin(state1Roll-refRoll)))/iyy)

    uPitch=(K[1,2]*(state1Pitch-refPitch))+((K[1,3]*state2Pitch))
    APitch=degrees((-uPitch+(m*g*l*sin(state1Pitch-refPitch)))/ixx)

    # print("uPitch:",uPitch)
    # print("APitch:",APitch)
    if base==-1: #jika tumpuan kaki kiri
        
        deltaRoll=((degrees(state2Roll))*tSmpl)+(ARoll*tSmpl*tSmpl/2)
        deltaPitch=((degrees(state2Pitch))*tSmpl)+(APitch*tSmpl*tSmpl/2)

        # controlDict["IAEY"]+=abs(deltaRoll)
        # controlDict["IAEX"]+=abs(deltaPitch)
        print("deltaRoll:",deltaRoll)
        print("deltaPitch:",deltaPitch)

        # swngPlan["tbase"]=deltaPitch

        #left leg (support)
        invPttrn["t18"]=dxl[17].prevGoalDegree-deltaRoll
        invPttrn["t16"]=dxl[15].prevGoalDegree-deltaPitch
        v18=abs(degrees(state2Roll)+(ARoll*tSmpl))
        v16=abs(degrees(state2Pitch)+(APitch*tSmpl))
        invPttrn["t10"]=0 #hip roll
        #right leg (swing)
        invPttrn["t17"]=invPttrn["t18"] #base roll
        invPttrn["t9"]=invPttrn["t18"] #hip roll
        # print("t9 kirim:",invPttrn["t9"])

        # print("t18roll:",invPttrn["t18"])
        # print("t16pitch",invPttrn["t16"])
        # print("v18:",v18)
        # print("v16:",v16)
        # print("delta_16_pitch",)
        invers_walk2(robot,dxl,'ki',pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],tSmpl,v18,v16,condition='walk2')
    
    #------------------------------------------------------------------------------------------
    elif base==1: #jika tumpuan kaki kanan

        deltaRoll=((degrees(state2Roll))*tSmpl)+(ARoll*tSmpl*tSmpl/2)
        deltaPitch=((degrees(state2Pitch))*tSmpl)+(APitch*tSmpl*tSmpl/2)
        # controlDict["IAEY"]+=abs(deltaRoll)
        # controlDict["IAEX"]+=abs(deltaPitch)

        # swngPlan["tbase"]=deltaPitch

        # print("deltaRoll:",deltaRoll)
        # print("deltaPitch:",deltaPitch)

        #right leg(support) 
        invPttrn["t17"]=dxl[16].prevGoalDegree-deltaRoll #base roll
        invPttrn["t15"]=dxl[14].prevGoalDegree+deltaPitch #base pitch
        invPttrn["t9"]=0 #hip roll
        v17=abs(degrees(state2Roll)+(ARoll*tSmpl))
        v15=abs(degrees(state2Pitch)+(APitch*tSmpl))
        #left leg (swing)
        invPttrn["t18"]=invPttrn["t17"] #base roll
        invPttrn["t10"]=invPttrn["t17"] #hip roll

        # print("t17:",invPttrn["t17"])
        # print("t15:",invPttrn["t15"])

        # print("t10 kirim:",invPttrn["t10"])
        invers_walk2(robot,dxl,'ka',pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],tSmpl,v17,v15,condition='walk2')
           
    if condition=='normal':
        robot.syncWrite() 
    elif condition=='virtual':
        pass

def Controlrey(robot,dxl,base,t,K,condition='normal'):
        #Control kendali roll pitch

    t=t/1000000 # ubah t dari microsecond ke second
    tServo=0.1
    tSmpl=0.1

    m=1.634
    g=9.80665
    l=0.19614907857545497
    ixx=0.092026292
    iyy=0.087070843
    bstate1=0.0
    bstate2=0.0

    #ubah status com ke sudut dan kecepatan sudut
    state1Roll=arctan(comNow["y"]/comNow["z"]) #sudut
    state2Roll=(state1Roll-controlDict["rollBef"])/tSmpl #kecepatan sudut

    state1Pitch=arctan(comNow["x"]/comNow["z"]) #sudut
    state2Pitch=(state1Pitch-controlDict["pitchBef"])/tSmpl #kecepatan sudut

    # print("state1Roll:",state1Roll)
    print("state1Pitch:",state1Pitch)
    print("state2Pitch:",state2Pitch)

    controlDict["pitchBef"],controlDict["rollBef"]=state1Pitch,state1Roll

    # #referensi
    refPitch=arctan(pttrn["Xt"]/comNow["z"])
    refRoll=arctan(pttrn["Yt"]/comNow["z"])
    # print("refRoll:",refRoll)
    # print("refPitch:",refPitch)

    #control parameter
    uRoll=(K[0,0]*(state1Roll-refRoll))+((K[0,1]*state2Roll))
    ARoll=degrees((-uRoll+(m*g*l*sin(state1Roll-refRoll)))/iyy)

    uPitch=(K[1,2]*(state1Pitch-refPitch))+((K[1,3]*state2Pitch))
    APitch=degrees((-uPitch+(m*g*l*sin(state1Pitch-refPitch)))/ixx)

    # print("uPitch:",uPitch)
    # print("APitch:",APitch)
    if base==-1: #jika tumpuan kaki kiri
        
        deltaRoll=((degrees(state2Roll))*tSmpl)+(ARoll*tSmpl*tSmpl/2)
        deltaPitch=((degrees(state2Pitch))*tSmpl)+(APitch*tSmpl*tSmpl/2)

        print("deltaRoll:",deltaRoll)
        print("deltaPitch:",deltaPitch)

        #left leg (support)
        invPttrn["t18"]=dxl[17].prevGoalDegree-deltaRoll
        invPttrn["t16"]=dxl[15].prevGoalDegree-deltaPitch
        v18=abs(degrees(state2Roll)+(ARoll*tSmpl))
        v16=abs(degrees(state2Pitch)+(APitch*tSmpl))
        invPttrn["t10"]=0 #hip roll
        #right leg (swing)
        invPttrn["t17"]=invPttrn["t18"] #base roll
        invPttrn["t9"]=invPttrn["t18"] #hip roll
        # print("t9 kirim:",invPttrn["t9"])

        # print("t18roll:",invPttrn["t18"])
        # print("t16pitch",invPttrn["t16"])
        # print("v18:",v18)
        # print("v16:",v16)
        # print("delta_16_pitch",)
        invers_walkrey(robot,dxl,'ki',pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],tSmpl,v18,v16)
    
    #------------------------------------------------------------------------------------------
    elif base==1: #jika tumpuan kaki kanan

        deltaRoll=((degrees(state2Roll))*tSmpl)+(ARoll*tSmpl*tSmpl/2)
        deltaPitch=((degrees(state2Pitch))*tSmpl)+(APitch*tSmpl*tSmpl/2)
        # print("deltaRoll:",deltaRoll)
        # print("deltaPitch:",deltaPitch)

        #right leg(support) 
        invPttrn["t17"]=dxl[16].prevGoalDegree-deltaRoll #base roll
        invPttrn["t15"]=dxl[14].prevGoalDegree+deltaPitch #base pitch
        invPttrn["t9"]=0 #hip roll
        v17=abs(degrees(state2Roll)+(ARoll*tSmpl))
        v15=abs(degrees(state2Pitch)+(APitch*tSmpl))
        #left leg (swing)
        invPttrn["t18"]=invPttrn["t17"] #base roll
        invPttrn["t10"]=invPttrn["t17"] #hip roll

        # print("t17:",invPttrn["t17"])
        # print("t15:",invPttrn["t15"])

        # print("t10s kirim:",invPttrn["t10"])
        invers_walkrey(robot,dxl,'ka',pttrn["sfx"],pttrn["sfy"],pttrn["sfz"],tSmpl,v17,v15)
           
    # if condition=='normal':
    #     robot.syncWrite() 
    # elif condition=='virtual':
    #     pass

def tuningLQRdiskrit(condition):

    ###dengan baterai
    # m=1.725 #(kg)
    # g=9.80665 #m/s^2
    # l=0.19614907857545497 #meter
    # ixx=0.0920262919239
    # iyy=0.087070843434217
    # izz=0.00805112193405

    #tanpa baterai
    m=1.634 #(kg)
    g=9.80665 #m/s^2
    l=0.19614907857545497 #meter
    ixx=0.084143172
    iyy=0.079494614
    izz=0.00757168

    A = np.array([[0,1,0,0],[m*g*l/ixx, 0, 0, 0],[0, 0, 0, 1],[0 ,0, m*g*l/iyy, 0]])
    B = np.array([[0 ,0],[1/ixx ,0],[0,0],[0 ,1/iyy]])
    C = np.array([[1, 0, 0 ,0],[0, 0 ,0 ,0],[0, 0 ,1 ,0],[0, 0 ,0 ,0]])
    D = np.array([[0 ,0],[0, 0],[0,0],[0,0]])

    if condition=='walk':
        Q = np.array([[1400,0,0,0], #roll kiri
                    [0,1,0,0], 
                    [0,0,600,0], #pitch kiri
                    [0,0,0,1]])

    elif condition=='walk2':
        Q = np.array([[1100,0,0,0], #roll kanan
                    [0,0.1,0,0], 
                    [0,0,800,0], #pitch kanan
                    [0,0,0,1]])

    elif condition=='translation roll':
        Q = np.array([[230,0,0,0], #paling better di 230 0.07//// atau 230 0.01 0.001
                    [0,0.07,0,0], 
                    [0,0,1,0],
                    [0,0,0,1]])

    elif condition=='translation roll 1 kaki':
        Q = np.array([[230.964476,0,0,0],
                    [0,0.00085357467561,0,0],
                    [0,0,1,0],
                    [0,0,0,1]])
    
    elif condition=='translation pitch':
        Q = np.array([[1,0,0,0],
                    [0,1,0,0],
                    [0,0,57.99946458667,0],
                    [0,0,0,0.3859]])

    elif condition=='translation pitch 1 kaki':
        Q = np.array([[1000,0,0,0],
                    [0,10,0,0],
                    [0,0,10,0],
                    [0,0,0,1]])
    
    elif condition=='coba':
        Q = np.array([[6000,0,0,0], #roll
                    [0,0.1978,0,0],
                    [0,0,1000,0], #pitch
                    [0,0,0,0.5]])       

    R = np.array([[1,0],[0,1]])

    # ##ubah ke discrete dengan kembalian berupa state space method zoh
    # sys = signal.StateSpace(A, B, C, D)
    # sysd= sys.to_discrete(0.1)
    # print("sysd",sysd)

    ##ubah ke discrete dengan kembalian berupa A,B,C,D,dt methode zoh
    sysd=signal.cont2discrete((A,B,C,D),0.1)
    A,B,C,D=sysd[0],sysd[1],sysd[2],sysd[3]
    print(A,B,C,D)

    ##------------mencari K dari system discrete----------------------
    ## first, solve the ricatti equation
    P = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
    # compute the LQR gain
    K = np.matrix(scipy.linalg.inv(B.T*P*B+R)*(B.T*P*A))
    # print("K",K)
    #----------------------------------------------------------------

    # #-----------mencari K dari system continue-----------------------
    # K, S, E = ctl.lqr(A,B, Q, R)
    # #----------------------------------------------------------------
    return Q,K

def cntTransPitch(robot,dxl,base,K,t,condition='normal'):

    t=t/1000000 # ubah t dari microsecond ke second
    tServo=0.1
    tSmpl=0.1

    m=1.634 #tanpa baterai
    g=9.80665
    l=0.19614907857545497
    ixx=0.092026292
    iyy=0.087070843
    bstate1=0.0
    bstate2=0.0
    print("===========================================")
    print("t:",t)
    if condition=='normal':
        COM(robot,dxl,'ki',readAll_leg='base')
    elif condition=='virtual':
        COM(robot,dxl,'ki',readAll_leg='virtual')

    #ubah status com ke sudut dan kecepatan sudut
    state1Pitch=arctan(comNow["x"]/comNow["z"]) #sudut
    state2Pitch=(state1Pitch-controlDict["pitchBef"])/tSmpl #kecepatan sudut

    controlDict["pitchBef"]=state1Pitch

    #referensi
    refPitch=arctan(pttrn["Xt"]/comDef["zt"])

    #control parameter
    uPitch=(K[1,2]*(state1Pitch-refPitch))+((K[1,3]*state2Pitch-0))
    APitch=degrees((-uPitch+(m*g*l*sin(state1Pitch-refPitch)))/iyy)

    deltaPitch=((degrees(state2Pitch))*tSmpl)+(APitch*tSmpl*tSmpl/2)
    # print(deltaRoll)
        
    #left leg (support)
    invPttrn["t16"]=dxl[15].prevGoalDegree-deltaPitch #base pitch
    v16=abs(degrees(state2Pitch)+(APitch*tSmpl))
    
    #right leg (swing)
    invPttrn["t15"]=dxl[14].prevGoalDegree+deltaPitch #base pitch
    v15=abs(degrees(state2Pitch)+(APitch*tSmpl))

    invers_translasi_pitch(robot,dxl,'ki',0,0,0,tSmpl,v16,v15)
    
    # if condition=='normal':
    #     robot.syncWrite() 
    # elif condition=='virtual':
    #     pass

def cntTransRoll(robot,dxl,base,K,t,condition='normal'):
    t=t/1000000 # ubah t dari microsecond ke second
    tServo=0.1
    tSmpl=0.1

    m=1.634 #tanpa baterai
    g=9.80665
    l=0.19614907857545497
    ixx=0.092026292
    iyy=0.087070843
    bstate1=0.0
    bstate2=0.0
    print("===========================================")
    print("t:",t)
    if condition=='normal':
        COM(robot,dxl,'ki',readAll_leg='base')
    elif condition=='virtual':
        COM(robot,dxl,'ki',readAll_leg='virtual')

    #ubah status com ke sudut dan kecepatan sudut
    state1Roll=arctan(comNow["y"]/comNow["z"]) #sudut
    state2Roll=(state1Roll-controlDict["rollBef"])/tSmpl #kecepatan sudut

    controlDict["rollBef"]=state1Roll

    #referensi
    refRoll=arctan(pttrn["Yt"]/comDef["zt"])

    #control parameter
    uRoll=(K[0,0]*(state1Roll-refRoll))+((K[0,1]*state2Roll-0))
    ARoll=degrees((-uRoll+(m*g*l*sin(state1Roll-refRoll)))/iyy)

    deltaRoll=((degrees(state2Roll))*tSmpl)+(ARoll*tSmpl*tSmpl/2)
    # print(deltaRoll)
        
    #left leg (support)
    invPttrn["t18"]=dxl[17].prevGoalDegree-deltaRoll #base roll
    v18=abs(degrees(state2Roll)+(ARoll*tSmpl))
    invPttrn["t10"]=dxl[9].prevGoalDegree-deltaRoll #hip roll
    
    #right leg (swing)
    invPttrn["t17"]=invPttrn["t18"] #base roll
    v17=abs(degrees(state2Roll)+(ARoll*tSmpl))
    invPttrn["t9"]=invPttrn["t18"]#hip roll

    # print("t18",invPttrn["t18"])
    # print("t17",invPttrn["t17"])

    # print("v18:",v18)
    # print("v17:",v17)

    invers_translasi_roll(robot,dxl,'ki',0,0,0,tSmpl,v18,v17)
    
    # if condition=='normal':
    #     robot.syncWrite() 
    # elif condition=='virtual':
    #     pass

def invers_translasi_roll(robot,dxl,base,x,y,z,times,v18,v17):

    IK_w=rospy.ServiceProxy('compute_invers_walk',ComputeInversWalk)
    req=ComputeInversWalkRequest()
    
    req.base.data=base
    req.coordinatX.data=x
    req.coordinatY.data=y
    req.coordinatZ.data=z

    req.length_la.data=invPttrn["la_Ki"]
    req.t_EngkleBaseRoll.data=invPttrn["t18"]
    req.t_EngkleBasePitch.data=-invPttrn["t16"]

    resp=IK_w.call(req)
    angle = [float(x) for x in resp.angleServo.data.split(",")]
    t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18=angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11]

    dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
    dxl[9].moveSync(invPttrn["t10"],times,dxl[9].prevGoal,read=0)
    dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0)
    dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
    dxl[15].moveSync(t16,times,dxl[15].prevGoal,read=0)
    dxl[17].moveSync(invPttrn["t18"],v18,dxl[17].prevGoal,time_type='omega',read=0)

    dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
    dxl[8].moveSync(invPttrn["t9"],times,dxl[8].prevGoal,read=0)
    dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
    dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
    dxl[14].moveSync(t15,times,dxl[14].prevGoal,read=0)
    dxl[16].moveSync(invPttrn["t17"],v17,dxl[16].prevGoal,time_type='omega',read=0)

def invers_translasi_pitch(robot,dxl,base,x,y,z,times,v16,v15):

    IK_w=rospy.ServiceProxy('compute_invers_walk',ComputeInversWalk)
    req=ComputeInversWalkRequest()
    
    req.base.data=base
    req.coordinatX.data=x
    req.coordinatY.data=y
    req.coordinatZ.data=z

    req.length_la.data=invPttrn["la_Ki"]
    req.t_EngkleBaseRoll.data=invPttrn["t18"]
    req.t_EngkleBasePitch.data=-invPttrn["t16"]

    resp=IK_w.call(req)
    angle = [float(x) for x in resp.angleServo.data.split(",")]
    t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18=angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7],angle[8],angle[9],angle[10],angle[11]

    dxl[6].moveSync(t7,times,dxl[6].prevGoal,read=0)
    dxl[9].moveSync(t10,times,dxl[9].prevGoal,read=0)
    dxl[11].moveSync(t12,times,dxl[11].prevGoal,read=0) #pake biasa kalau pitch 1 kaki ini aja yg nyala
    dxl[13].moveSync(t14,times,dxl[13].prevGoal,read=0)
    dxl[15].moveSync(invPttrn["t16"],v16,dxl[15].prevGoal,time_type='omega',read=0) #kalau pitch 1 kaki ini aja yg nyala
    dxl[17].moveSync(t18,times,dxl[17].prevGoal,read=0)

    dxl[7].moveSync(t8,times,dxl[7].prevGoal,read=0)
    dxl[8].moveSync(t9,times,dxl[8].prevGoal,read=0)
    dxl[10].moveSync(t11,times,dxl[10].prevGoal,read=0)
    dxl[12].moveSync(t13,times,dxl[12].prevGoal,read=0)
    dxl[14].moveSync(invPttrn["t15"],times,dxl[14].prevGoal,time_type='omega',read=0)
    dxl[16].moveSync(t17,times,dxl[16].prevGoal,read=0)        