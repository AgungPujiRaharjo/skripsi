#!/usr/bin/env python3

import rospy
import math
from program.srv import ComputeInversWalk,ComputeInversWalkResponse
from numpy import sin,cos,pi,array,eye,dot,arccos,arcsin,arctan,radians

def invers_walk(base,x,y,z,length_la,t_EngkleBaseRoll,t_EngkleBasePitch):
    
    #panjang di gambar desain invers (berbeda dengan panjang di perhitungan forward)
    l1=3.313 #cm
    l2=7.5 #cm
    l3=7.5 
    l4=3.2

    if base=='ki':
        #----------------------kaki base----------------------------
        #sumbu x
        ha=length_la
        t16=t_EngkleBasePitch

        t1b=((l2**(2))+(ha**(2))-(l3**(2)))/(2*l2*ha)
        t1b=arccos(t1b)*180/pi
        tm=((l2**(2))+(l3**(2))-(ha**(2)))/(2*l2*l3)
        tm=arccos(tm)*180/pi
        t14=180-tm
        t14=t14-3.7996

        tn=180-t1b-tm
        to=tn-t16
        t1a=to
        t12=t1a+t1b
        t12=t12-3.7996

        t16=-t16

        xbase=ha*sin(radians(t1a))
        za=math.sqrt((ha**(2))-(xbase**(2)))
        zbase=l1+za+l4

        #sumbu y
        t18=t_EngkleBaseRoll
        t10=t18

        #sumbu z
        t7=0

        # print("t10:",t10)    
        # print("t12:",t12)
        # print("t14:",t14)
        # print("t16:",t16)
        # print("t18:",t18)
        # print("x:",round(xbase,3))

        #---------------------------------------kaki swing---------------------------------
        #sumbu x
        xswing=x+xbase
        zswing=zbase-z
        
        z1=l1
        z3=l4
        z2=zswing-z1-z3
        la=math.sqrt((z2**(2))+(xswing**(2)))
        tj=((l2**(2))+(la**(2))-(l2**(2)))/(2*l2*la)
        tj=arccos(tj)*180/pi
        tk=arcsin(xswing/la)*180/pi
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
        td=arccos(xswing/la)*180/pi
        lh=math.sqrt((xswing**(2))+(l4**(2)))
        te=arcsin(l4/lh)*180/pi
        tf=180-tn-td-te
        t15=90-te-tf #sudut theta 15

        #sumbu y
        t9=arcsin(y/(l2+l3))
        t9=t9*180/pi
        ta=arccos(y/(l2+l3))
        ta=ta*180/pi
        t17=180-90-ta

        #sumbu z
        t8=0

    elif base=='ka':
        #----------------------kaki base----------------------------
        #sumbu x
        ha=length_la
        t15=t_EngkleBasePitch

        t1b=((l2**(2))+(ha**(2))-(l3**(2)))/(2*l2*ha)
        t1b=arccos(t1b)*180/pi
        tm=((l2**(2))+(l3**(2))-(ha**(2)))/(2*l2*l3)
        tm=arccos(tm)*180/pi
        t13=180-tm
        t13=-t13
        t13=t13+3.7996

        tn=180-t1b-tm
        to=tn-t15
        t1a=to
        t11=t1a+t1b
        t11=-t11
        t11=t11+3.7996

        xbase=ha*sin(radians(t1a))
        za=math.sqrt((ha**(2))-(xbase**(2)))
        zbase=l1+za+l4

        #sumbu y
        t17=t_EngkleBaseRoll
        t9=t17

        #sumbu z
        t7=0

        # print("t10:",t10)    
        # print("t12:",t12)
        # print("t14:",t14)
        # print("t16:",t16)
        # print("t18:",t18)
        # print("x:",round(xbase,3))

        #---------------------------------------kaki swing---------------------------------
        #sumbu x
        xswing=x+xbase
        zswing=zbase-z

        z1=l1
        z3=l4
        z2=zswing-z1-z3
        la=math.sqrt((z2**(2))+(xswing**(2)))
        tj=((l2**(2))+(la**(2))-(l2**(2)))/(2*l2*la)
        tj=arccos(tj)*180/pi
        tk=arcsin(xswing/la)*180/pi
 
        t12=tj+tk #sudut theta 12

        t12=t12-3.7996

        tm=((l2**(2))+(l3**(2))-(la**(2)))/(2*l2*l3)
        tm=arccos(tm)*180/pi
        t14=180-tm #sudut theta 14

        t14=t14-3.7996

        tn=((la**(2))+(l3**(2))-(l2**(2)))/(2*la*l3)
        tn=arccos(tn)*180/pi
        td=arccos(xswing/la)*180/pi
        lh=math.sqrt((xswing**(2))+(l4**(2)))
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

        t8=0

    angle="%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f" % (t7,t8,t9,t10,t11,t12,t13,t14,t15,t16,t17,t18)

    return angle

def server_callback(req):
    rospy.loginfo("receive request to compute invers_walk")
    resp=ComputeInversWalkResponse()

    base=req.base.data
    x=req.coordinatX.data
    y=req.coordinatY.data
    z=req.coordinatZ.data
    length_la=req.length_la.data
    t_EngkleBaseRoll=req.t_EngkleBaseRoll.data
    t_EngkleBasePitch=req.t_EngkleBasePitch.data
   
    result=invers_walk(base,x,y,z,length_la,t_EngkleBaseRoll,t_EngkleBasePitch)

    resp.angleServo.data=result
    
    return resp

rospy.init_node("compute_invers_walk_server",anonymous=False)
rospy.Service("compute_invers_walk",ComputeInversWalk,server_callback)
rospy.spin()
