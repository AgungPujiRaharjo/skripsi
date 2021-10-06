#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from program.srv import GetUltrasonicData,GetUltrasonicDataResponse

dataultrasonic={"jarak10":0}

def callback_ultrasonic(msg):
    data = float(msg.data)
    dataultrasonic["jarak10"] = data
    # print(msg)

def service_callback(req):
    rospy.loginfo("receive request to get ultrasonic data")
    resp=GetUltrasonicDataResponse()

    resp.ultrasonic_data.data="%f" % (dataultrasonic["jarak10"])
    return resp

rospy.init_node('ultrasonic_server',anonymous=True)
rospy.Subscriber('ultrasonic_data',String,callback_ultrasonic)
rospy.Service("get_ultrasonic_data",GetUltrasonicData,service_callback)
rospy.spin()

