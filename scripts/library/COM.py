#!/usr/bin/env python3

import rospy
from numpy import sin,cos,pi,array,eye,dot,arccos,arcsin,arctan
from numpy.linalg import multi_dot
from program.srv import ComputeCOM,ComputeCOMResponse

#dalam mm (acuan perhitungan DH)
l1=56.68
l2=32
l3=75
l4=14.87
l5=14.13
l6=60.87
l7=33.13
l7a=19
l8=76.64

#Massa part (gram) - acuan massa perhitungan COM
mBase=36
mEngkel=129
mKakiBawah=82
mKakiAtas=30
mHip=129
mTempurung=15
mBadanTangan=802 #893 pake baterai

matriks_fwd=[]

def forwardLeg(base,angleLeg,nFrame=27):
 
    if base=='ki': #tumpuan kaki kiri
        t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18=angleLeg[0],angleLeg[1],angleLeg[2],-angleLeg[3],angleLeg[4],-angleLeg[5],angleLeg[6],-angleLeg[7],angleLeg[8],-angleLeg[9],angleLeg[10],-angleLeg[11]

        #DH parameter (v3)
        t=[0,0+90,t18-90,t16,0,t14,90,-90,t12,0,t10,-90,0,t8,t7,90,-90,t9+90,t11,-90,90,t13-90,90,t15,0,t17,0]
        alp=[0,-90,-90,-90,90,0,0,180,-90,180,0,-90,180,0,-180,0,90,90,0,0,180,0,0,90,180,0,-90]
        d=[0,l2,-l1,0,-l4,0,0,0,0,l1,0,l1,l7a,0,l7a,0,-l7,l1,0,0,0,0,0,0,-l1,0,-l1]
        a=[-l1,0,0,l3,0,l5,l4,l6,0,0,l7,0,0,-l8,0,l1,0,0,-l6,l4,-l5,l4,-l3,0,0,-l2,0]

    if base=='ka': #tumpuan kaki kanan
        t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18=angleLeg[0],angleLeg[1],-angleLeg[2],angleLeg[3],-angleLeg[4],angleLeg[5],-angleLeg[6],angleLeg[7],-angleLeg[8],angleLeg[9],-angleLeg[10],angleLeg[11]
       
        #DH parameter(v3)
        t=[0,0+90,t17-90,t15,0,t13,-90,90,t11,0,t9,-90,0,t7,t8,90,-90,t10+90,t12,-90,90,t14-90,90,t16,0,t18,0]
        alp=[0,-90,90,-90,90,0,0,180,90,180,0,-90,180,0,-180,0,90,-90,180,0,0,0,0,90,0,0,0]
        d=[0.00,l2,-l1,0,l4,0,0,0,0,l1,0,l1,l7a,0,l7a,0,-l7,l1,0,0,0,0,0,0,l1,0,-l1]
        a=[-l1,0,0,l3,0,l5,l4,l6,0,0,l7,0,0,l8,0,l1,0,0,-l6,l4,-l5,-l4,-l3,0,0,-l2,0]  

    #ubah semua t ke radian
    for i in range(nFrame):
        t[i]=float(t[i])/180*pi

    #ubah semua alpha ke radian
    for i in range(nFrame):
        alp[i]=float(alp[i])/180*pi

    trans=[0. for ka in range(nFrame)]
    Tot=eye(4) #matriks identitas

    #perkalian matriks ke n frame
    for i in range(nFrame):
        trans[i]=array([[cos(t[i]),-sin(t[i])*cos(alp[i]),sin(t[i])*sin(alp[i]),a[i]*cos(t[i])],
                    [sin(t[i]),cos(t[i])*cos(alp[i]),-cos(t[i])*sin(alp[i]),a[i]*sin(t[i])],
                    [0,sin(alp[i]),cos(alp[i]),d[i]],
                    [0,0,0,1]])
        # print("trans",i,":",trans[i])
        Tot=dot(Tot,trans[i])
        matriks_fwd.append(Tot)
        # print("until:",i)
        # print("Tot:",Tot)

    return Tot

def Lok_Trans(base,nFrame,nLok):
    if base=='ki':
        #letak com lokal part
        lokX=[-4.71,12.68,-12.56560976,-43.41,-12.85, 0    ,38.32  ,-39.25,12.68,18.79,62.16914634 ,-12.85,7.05]
        lokY=[  0  ,-15.9,-0.652439024,1.06  , 0.39 ,16.48 ,-14    ,0     ,-15.9,-1.06,0.992317073 ,-0.39 ,4.71]
        lokZ=[7.05 ,0    ,-0.89445122 ,0     , 15.81,-10.65,-52.042,-10.65,0    ,0    ,-1          ,-15.81,0   ]
    elif base=='ka':
        #letak com lokal part
        lokX=[-4.71,12.68,-12.56560976,-43.41,-12.85, 0    ,-38.32  ,-39.25,12.68,18.79,62.16914634 ,-12.85,7.05]
        lokY=[  0  ,15.9 ,0.652439024 ,-1.06 , 0.39 ,16.48 ,-14     ,0     ,15.9 ,-1.06,-0.992317073,-0.39 ,0]
        lokZ=[7.05 ,0    ,0.89445122  ,0     , 15.81,-10.65,-52.042,-10.65,0    ,0    ,1            ,-15.81,-4.71]

    #matriks com lokal part
    lokal=array([[lokX[nLok]],
                [lokY[nLok]],
                [lokZ[nLok]],
                [1]])
    
    MultLokTrans=dot(matriks_fwd[nFrame],lokal) 

    return MultLokTrans

def COM(base,angle):
    matriks_fwd.clear()
    angleLeg = [int(x) for x in angle.split(",")]

    forwardLeg(base,angleLeg)

    #lokal0 - Telapak kaki/base kaki support
    xBaseSupport=-4.71
    yBaseSupport=0
    zBaseSupport=7.05

    #lokal1(n0-2) - Engkel kaki support
    result=Lok_Trans(base,2,1)
    xEngkelSupport= result[0,0] 
    yEngkelSupport= result[1,0]
    zEngkelSupport= result[2,0]

    #lokal2(n0-3) - kaki bagian bawah kaki support
    result=Lok_Trans(base,3,2)
    xBawahSupport= result[0,0]
    yBawahSupport= result[1,0]
    zBawahSupport= result[2,0]

    #lokal3(n0-7) - kaki bagian atas kaki support
    result=Lok_Trans(base,7,3)
    xAtasSupport= result[0,0]
    yAtasSupport= result[1,0]
    zAtasSupport= result[2,0]
 
    #lokal4(n0-9) - HIP kaki support
    result=Lok_Trans(base,9,4)
    xHipSupport= result[0,0]
    yHipSupport= result[1,0]
    zHipSupport= result[2,0]

    #lokal5(n0-11) - Tempurung Hip kaki support
    result=Lok_Trans(base,11,5)
    xTmpSupport= result[0,0]
    yTmpSupport= result[1,0]
    zTmpSupport= result[2,0]

    #lokal6(n0-11b) - Badan
    result=Lok_Trans(base,13,6)
    xBadan= result[0,0]
    yBadan= result[1,0]
    zBadan= result[2,0]

    #lokal7(n0-13) - Tempurung Hip kaki swing
    result=Lok_Trans(base,15,7)
    xTmpSwing= result[0,0]
    yTmpSwing= result[1,0]
    zTmpSwing= result[2,0]

    #lokal8(n0-15) - HIP kaki swing
    result=Lok_Trans(base,17,8)
    xHipSwing= result[0,0]
    yHipSwing= result[1,0]
    zHipSwing= result[2,0]

    #lokal9(n0-16) - kaki bagian atas kaki swing
    result=Lok_Trans(base,18,9)
    xAtasSwing= result[0,0]
    yAtasSwing= result[1,0]
    zAtasSwing= result[2,0]

    #lokal10(n0-20) - kaki bagian bawah kaki swing
    result=Lok_Trans(base,22,10)
    xBawahSwing= result[0,0]
    yBawahSwing= result[1,0]
    zBawahSwing= result[2,0]

    #lokal11(n0-22) - Engkel kaki swing
    result=Lok_Trans(base,24,11)
    xEngkelSwing= result[0,0]
    yEngkelSwing= result[1,0]
    zEngkelSwing= result[2,0]

    #lokal12(n0-24) - Telapak kaki swing
    result=Lok_Trans(base,26,12)
    xBaseSwing= result[0,0]
    yBaseSwing= result[1,0]
    zBaseSwing= result[2,0]
    
    #com
    comX=( (xBaseSupport*mBase)+(xEngkelSupport*mEngkel)+(xBawahSupport*mKakiBawah)+(xAtasSupport*mKakiAtas)+(xHipSupport*mHip)+(xTmpSupport*mTempurung)+(xBadan*mBadanTangan)+(xTmpSwing*mTempurung)+(xHipSwing*mHip)+(xAtasSwing*mKakiAtas)+(xBawahSwing*mKakiBawah)+(xEngkelSwing*mEngkel)+(xBaseSwing*mBase) ) / ( (2*mBase)+(2*mEngkel)+(2*mKakiBawah)+(2*mKakiAtas)+(2*mHip)+(2*mTempurung)+mBadanTangan )
    comY=( (yBaseSupport*mBase)+(yEngkelSupport*mEngkel)+(yBawahSupport*mKakiBawah)+(yAtasSupport*mKakiAtas)+(yHipSupport*mHip)+(yTmpSupport*mTempurung)+(yBadan*mBadanTangan)+(yTmpSwing*mTempurung)+(yHipSwing*mHip)+(yAtasSwing*mKakiAtas)+(yBawahSwing*mKakiBawah)+(yEngkelSwing*mEngkel)+(yBaseSwing*mBase) ) / ( (2*mBase)+(2*mEngkel)+(2*mKakiBawah)+(2*mKakiAtas)+(2*mHip)+(2*mTempurung)+mBadanTangan )
    comZ=( (zBaseSupport*mBase)+(zEngkelSupport*mEngkel)+(zBawahSupport*mKakiBawah)+(zAtasSupport*mKakiAtas)+(zHipSupport*mHip)+(zTmpSupport*mTempurung)+(zBadan*mBadanTangan)+(zTmpSwing*mTempurung)+(zHipSwing*mHip)+(zAtasSwing*mKakiAtas)+(zBawahSwing*mKakiBawah)+(zEngkelSwing*mEngkel)+(zBaseSwing*mBase) ) / ( (2*mBase)+(2*mEngkel)+(2*mKakiBawah)+(2*mKakiAtas)+(2*mHip)+(2*mTempurung)+mBadanTangan )
    comX=round(comX,3)
    comY=round(comY,3)
    comZ=round(comZ,3)
    
    return comX,comY,comZ

def server_callback(req):
    rospy.loginfo("receive request to compute COM")
    resp=ComputeCOMResponse()

    result=COM(req.base.data,req.angle.data)

    xFwd=matriks_fwd[req.nFwd.data][0,3]
    yFwd=matriks_fwd[req.nFwd.data][1,3]
    zFwd=matriks_fwd[req.nFwd.data][2,3]

    resp.resp_forward.data="%f,%f,%f" %(xFwd,yFwd,zFwd)
    resp.comX.data=result[0]
    resp.comY.data=result[1]
    resp.comZ.data=result[2]
    
    return resp

rospy.init_node("compute_COM_server",anonymous=False)
rospy.Service("compute_COM",ComputeCOM,server_callback)
rospy.spin()

