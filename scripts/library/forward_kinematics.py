#!/usr/bin/env python3

import rospy
from numpy import sin,cos,pi,array,eye,dot,arccos,arcsin,arctan
from numpy.linalg import multi_dot
from program.srv import ComputeForwardKinematics,ComputeForwardKinematicsResponse

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

def forward(base,angle,nFrame):
    angleLeg = [int(x) for x in angle.split(",")]

    if base=='ki': #tumpuan kaki kiri
        t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18=angleLeg[0],angleLeg[1],angleLeg[2],-angleLeg[3],angleLeg[4],-angleLeg[5],angleLeg[6],-angleLeg[7],angleLeg[8],-angleLeg[9],angleLeg[10],-angleLeg[11]

        #DH parameter (v3)
        t=[0,0+90,t18-90,t16,0,t14,90,-90,t12,0,t10,-90,0,t7,t8,90,-90,t9+90,t11,-90,90,t13-90,90,t15,0,t17,0]
        alp=[0,-90,-90,-90,90,0,0,180,-90,180,0,-90,180,0,-180,0,90,90,0,0,180,0,0,90,180,0,-90]
        d=[0,l2,-l1,0,-l4,0,0,0,0,l1,0,l1,l7a,0,l7a,0,-l7,l1,0,0,0,0,0,0,-l1,0,-l1]
        a=[-l1,0,0,l3,0,l5,l4,l6,0,0,l7,0,0,-l8,0,l1,0,0,-l6,l4,-l5,l4,-l3,0,0,-l2,0]

    if base=='ka': #tumpuan kaki kanan
        t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18=angleLeg[0],angleLeg[1],-angleLeg[2],angleLeg[3],-angleLeg[4],angleLeg[5],-angleLeg[6],angleLeg[7],-angleLeg[8],angleLeg[9],-angleLeg[10],angleLeg[11]
       
        #DH parameter(v3)
        t=[0,0+90,t17-90,t15,0,t13,-90,90,t11,0,t9,-90,0,t8,t7,90,-90,t10+90,t12,-90,90,t14-90,90,t16,0,t18,0]
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
    Tot=eye(4)

    #perkalian matriks ke n frame
    for i in range(nFrame):
        trans[i]=array([[cos(t[i]),-sin(t[i])*cos(alp[i]),sin(t[i])*sin(alp[i]),a[i]*cos(t[i])],
                    [sin(t[i]),cos(t[i])*cos(alp[i]),-cos(t[i])*sin(alp[i]),a[i]*sin(t[i])],
                    [0,sin(alp[i]),cos(alp[i]),d[i]],
                    [0,0,0,1]])
        # print("trans",i,":",trans[i])
        Tot=dot(Tot,trans[i])

    return Tot

def server_callback(req):
    rospy.loginfo("receive request to compute forward leg")
    resp=ComputeForwardKinematicsResponse()

    result=forward(req.base.data,req.angle.data,req.nFrame.data)
    
    resp.resultX.data=result[0,3]
    resp.resultY.data=result[1,3]
    resp.resultZ.data=result[2,3]

    return resp

rospy.init_node("compute_forward_kinematics_server",anonymous=False)
rospy.Service("compute_forward_kinematics",ComputeForwardKinematics,server_callback)
rospy.spin()
