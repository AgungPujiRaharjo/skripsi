#!/usr/bin/env python3
from servo import *
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from array import *
import rospy
from dynamixel_sdk_examples.srv import *

class Robot:

    def __init__(self,object_dxl,port_handler,packet_handler,sync_write):
        self.dxl = object_dxl
        self.portHandler = port_handler
        self.packetHandler = packet_handler
        self.groupSyncWrite = sync_write

    def cekServo(self,addr_AX_torque, data):
        for obj in self.dxl:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, obj.id, addr_AX_torque, data)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % obj.id)

    def readOne(self,IDdxl,fromDef='yes',data='int'):
        addr_AX_Pres_pos = 36
        self.dxl[IDdxl-1].presentpos, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.dxl[IDdxl-1].id, addr_AX_Pres_pos)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # print("present position id %s is %s in degrees %f" %(self.dxl[IDdxl-1].id,self.dxl[IDdxl-1].presentpos,(self.dxl[IDdxl-1].presentpos*0.293)))
        
        if fromDef=='yes':
            if data=='int':
                angle=int(round((self.dxl[IDdxl-1].presentpos*0.293)-(self.dxl[IDdxl-1].default*0.293)))
            elif data=='float':
                angle=float(round((self.dxl[IDdxl-1].presentpos*0.293)-(self.dxl[IDdxl-1].default*0.293),3))
        else:
            angle=int(round(self.dxl[IDdxl-1].presentpos*0.293))
        return angle
    
    def rosReadOne(self,IDdxl,fromDef='yes'):
        rospy.wait_for_service('get_position')
        read_angle=rospy.ServiceProxy('get_position',GetPosition)
        req=GetPositionRequest()
        req.id=IDdxl
        resp=read_angle.call(req)
        self.dxl[IDdxl-1].presentpos=resp.position
        
        if fromDef=='yes':
            angle=int(round((self.dxl[IDdxl-1].presentpos*0.293)-(self.dxl[IDdxl-1].default*0.293)))
        else:
            angle=int(round(self.dxl[IDdxl-1].presentpos*0.293))
        return angle

    def readHand(self,fromDef='yes'):
        angleHand=[]
        addr_AX_Pres_pos = 36
        for obj in range(0,6):
            self.dxl[obj].presentpos, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.dxl[obj].id, addr_AX_Pres_pos)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            # print("present position id %s is %s in degrees %f" %(self.dxl[obj].id,self.dxl[obj].presentpos,(self.dxl[obj].presentpos*0.293)))
            
            if fromDef=='yes':
                angleHand.append(int(round((self.dxl[obj].presentpos*0.293)-(self.dxl[obj].default*0.293))))
            else:
                angleHand.append(int(round(self.dxl[obj].presentpos*0.293)))
        return angleHand

    def readLeg(self,fromDef='yes'):
        # angleLeg=[0,0,0,0,0,0,0,0,0,0,0,90]
        angleLeg=[]
        addr_AX_Pres_pos = 36
        for obj in range(6,18):
            self.dxl[obj].presentpos, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.dxl[obj].id, addr_AX_Pres_pos)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            # print("present position id %s is %s in degrees %f" %(self.dxl[obj].id,self.dxl[obj].presentpos,(self.dxl[obj].presentpos*0.293)))
            
            if fromDef=='yes':
                angleLeg.append(int(round((self.dxl[obj].presentpos*0.293)-(self.dxl[obj].default*0.293))))
            else:
                angleLeg.append(int(round(self.dxl[obj].presentpos*0.293)))
        return angleLeg

    def rosReadLeg(self,fromDef='yes'):
        # angleLeg=[0,0,0,0,0,0,0,0,0,0,0,90]
        angleLeg=[]
        addr_AX_Pres_pos = 36
        for obj in range(6,18):
            rospy.wait_for_service('get_position')
            read_angle=rospy.ServiceProxy('get_position',GetPosition)
            req=GetPositionRequest()
            req.id=self.dxl[obj].id
            resp=read_angle.call(req)
            self.dxl[obj].presentpos=resp.position
            if fromDef=='yes':
                angleLeg.append(int(round((self.dxl[obj].presentpos*0.293)-(self.dxl[obj].default*0.293))))
            else:
                angleLeg.append(int(round(self.dxl[obj].presentpos*0.293)))
        return angleLeg

    def readAllPresentPos(self):
        readAllAngle=[]
        addr_AX_Pres_pos = 36
        for obj in self.dxl:
            obj.presentpos, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, obj.id, addr_AX_Pres_pos)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            # print("present position id %s is %s in degrees %f" %(obj.id,obj.presentpos,(obj.presentpos*0.293)))
            readAllAngle.append(round(obj.presentpos*0.293))
        return readAllAngle
        
    def readAll(self,address,length):
        out = []
        for obj in self.dxl:
            if length == 2 :
                feedback, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, obj.id, address)
            if length == 1 :
                feedback, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, obj.id, address)
            out.append(feedback)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        return out

    def syncWrite(self):
        for obj in self.dxl:
            dxl_addparam_result = self.groupSyncWrite.addParam(obj.id, obj.param)
            # print(obj.id, ": ", obj.param)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % obj.id)
                print("return: ", dxl_addparam_result)

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()

