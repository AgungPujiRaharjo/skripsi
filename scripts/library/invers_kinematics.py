#!/usr/bin/env python3

import rospy
import math
from program.srv import ComputeInversKinematics,ComputeInversKinematicsResponse
from numpy import sin,cos,pi,array,eye,dot,arccos,arcsin,arctan

#info : hmax saat default -> 213,3 mm atau 21,33 cm
#panjang di gambar desain invers (berbeda dengan panjang di perhitungan forward)
l1=3.313 #cm
l2=7.5 #cm
l3=7.5 
l4=3.2

def invers(base,x,y,z): # x,y,z dalam cm ; times detik

    #kaki kiri
    if base=='ki':
        
        #sumbu x
        z1=l1
        z3=l4
        z2=z-z1-z3
        la=math.sqrt((z2**(2))+(x**(2)))
        tj=((l2**(2))+(la**(2))-(l3**(2)))/(2*l2*la)
        tj=arccos(tj)*180/pi
        tk=arcsin(x/la)*180/pi
 
        t12=tj+tk #sudut theta 12

        t12=t12-3.7996      #menyesuaikan sudutnya dengan mengurangi 4 derajat

        tm=((l2**(2))+(l3**(2))-(la**(2)))/(2*l2*l3)
        tm=arccos(tm)*180/pi
        t14=180-tm #sudut theta 14

        t14=t14-3.7996  #menyesuaikan sudutnya dengan mengurangi 4 derajat

        tn=((la**(2))+(l3**(2))-(l2**(2)))/(2*la*l3)
        tn=arccos(tn)*180/pi
        td=arccos(x/la)*180/pi
        lh=math.sqrt((x**(2))+(l4**(2)))
        te=arcsin(l4/lh)*180/pi
        tf=180-tn-td-te
        t16=90-te-tf #sudut theta 16
        t16=-t16

        #sumbu y
        t10=arcsin(y/(l2+l3))
        t10=t10*180/pi
        tb=arccos(y/(l2+l3))
        tb=tb*180/pi
        t18=180-90-tb

        t7=0
        # print("t7:",t7)
        # print("t10:",t10)    
        # print("t12:",t12)
        # print("t14:",t14)
        # print("t16:",t16)
        # print("t18:",t18)
        # print("la:,la")

        angle="%f,%f,%f,%f,%f,%f" % (t7,t10,t12,t14,t16,t18)
                   
    #kaki kanan
    if base=='ka':

        #sumbu x
        z1=l1
        z3=l4
        z2=z-z1-z3
        la=math.sqrt((z2**(2))+(x**(2)))
        tj=((l2**(2))+(la**(2))-(l2**(2)))/(2*l2*la)
        tj=arccos(tj)*180/pi
        tk=arcsin(x/la)*180/pi
        t11=tj+tk #sudut theta 11
        t11=-t11

        t11=t11+3.7996

        tm=((l2**(2))+(l3**(2))-(la**(2)))/(2*l2*l3)
        tm=arccos(tm)*180/pi
        t13=180-tm #sudut theta 13
        t13=-t13

        t13=t13+3.7996

        tn=((la**(2))+(l3**(2))-(l2**(2)))/(2*la*l3)
        tn=arccos(tn)*180/pi
        td=arccos(x/la)*180/pi
        lh=math.sqrt((x**(2))+(l4**(2)))
        te=arcsin(l4/lh)*180/pi
        tf=180-tn-td-te
        t15=90-te-tf #sudut theta 15

        #sumbu y
        t9=arcsin(y/(l2+l3))
        t9=t9*180/pi
        ta=arccos(y/(l2+l3))
        ta=ta*180/pi
        t17=180-90-ta

        t8=0
        # print("t8:",t8)
        # print("t9:",t9)
        # print("t11:",t11)
        # print("t13:",t13)
        # print("t15:",t15)
        # print("t17:",t17)
        
        angle="%f,%f,%f,%f,%f,%f" % (t8,t9,t11,t13,t15,t17)
           
    return angle,la

def server_callback(req):
    rospy.loginfo("receive request to compute invers")
    resp=ComputeInversKinematicsResponse()

    base=req.base.data
    x=req.coordinatX.data
    y=req.coordinatY.data
    z=req.coordinatZ.data
   
    result=invers(base,x,y,z)

    resp.angleServo.data=result[0]
    resp.lengthLA.data=float(result[1])
    
    return resp

rospy.init_node("compute_invers_kinematics_server",anonymous=False)
rospy.Service("compute_invers_kinematics",ComputeInversKinematics,server_callback)
rospy.spin()
