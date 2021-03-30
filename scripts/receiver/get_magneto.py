#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from program.srv import GetMagnetoData,GetMagnetoDataResponse

dataMagneto={}

def callback(msg):
    data = [float(x) for x in msg.data.split(",")]
    dataMagneto["heading"]=data[0]
    dataMagneto["time"]=data[1]

def service_callback(req):
    rospy.loginfo("receive request to get magneto data")
    resp=GetMagnetoDataResponse()

    resp.magneto_data.data="%f,%f" % (dataMagneto["heading"],dataMagneto["time"])
    return resp

rospy.init_node("magneto_server",anonymous=True)
rospy.Subscriber("magneto_data",String,callback)
rospy.Service("get_magneto_data",GetMagnetoData,service_callback)
rospy.spin()
