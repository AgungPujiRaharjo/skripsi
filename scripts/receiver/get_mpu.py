#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from program.srv import GetMpuData,GetMpuDataResponse

dataMpu={}

def callback(msg):
    data = [float(x) for x in msg.data.split(",")]
    dataMpu["roll"]=data[0]
    dataMpu["pitch"]=data[1]

def service_callback(req):
    rospy.loginfo("receive request to get mpu data")
    resp=GetMpuDataResponse()

    resp.mpu_data.data="%f,%f" % (dataMpu["roll"],dataMpu["pitch"])
    return resp

rospy.init_node("mpu_server",anonymous=True)
rospy.Subscriber("mpu_data",String,callback)
rospy.Service("get_mpu_data",GetMpuData,service_callback)
rospy.spin()