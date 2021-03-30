from dynamixel_sdk import * 
from numpy import pi
import csv
class Servo:
    # presentpos = 0
    ADDR_AX_PRES_POS               = 36
    ADDR_AX_GOAL_POSITION          = 30
    ADDR_AX_SPEED                  = 32

    def __init__(self, default, id, port_handler, packet_handler):
        self.default = default
        self.id = id
        self.PortHandler = port_handler
        self.PacketHandler = packet_handler
    
    def read(self,address,length):
        # out = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        out = []
        if length == 2 :
            feedback, dxl_comm_result, dxl_error = self.PacketHandler.read2ByteTxRx(self.PortHandler, self.id, address)
        if length == 1 :
            feedback, dxl_comm_result, dxl_error = self.PacketHandler.read1ByteTxRx(self.PortHandler, self.id, address)
        out.append(feedback)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.PacketHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.PacketHandler.getRxPacketError(dxl_error))
        # print ("Position: ", out)
        return out
         
    def move(self,regDegree,time,type='reg'):
#        if self.first_move == 0:
#            prev = self.read(self.ADDR_AX_PRES_POS,2)
#            self.prevGoal = prev[0]
#            self.first_move == 1
        if type == 'degree':
            degree = (self.default*0.293)+regDegree
            degree2Reg = degree/0.293
            degree = int(degree2Reg)
        if type == 'reg':
            degree = regDegree

        prev = self.read(self.ADDR_AX_PRES_POS,2)
        self.presentpos = prev[0]
#        self.presentpos = self.prevGoal        
        difference = abs(self.presentpos - degree) * 0.293
        self.speed = difference/(time*0.666)
        self.prevGoal = degree
        self.speed = int(self.speed) + 5
        # if speed < 7 :
            # speed = 0
        dxl_comm_result, dxl_error = self.PacketHandler.write2ByteTxRx(self.PortHandler, self.id, self.ADDR_AX_GOAL_POSITION, int(degree))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.PacketHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.PacketHandler.getRxPacketError(dxl_error))
        
        dxl_comm_result, dxl_error = self.PacketHandler.write2ByteTxRx(self.PortHandler, self.id, self.ADDR_AX_SPEED, int(self.speed))
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.PacketHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.PacketHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d move" % self.id)
        
        
    def moveSync(self,regDegree,time,prevGoal_param=0,type='degree',time_type='sec',move_flag=1, read=1):
        
        self.posNow = 0
        self.prevGoal = prevGoal_param
        
#        self.first_move = move_flag
#        if self.first_move == 0:
#            prev = self.read(self.ADDR_AX_PRES_POS,2)
#            self.prevGoal = prev[0]
#            self.first_move == 1
#            print("Masuk read awal")
        if type == 'degree':
            self.prevGoalDegree=regDegree
            degree = (self.default*0.293)+regDegree
            degree2Reg = round(degree/0.293)
            degree = int(degree2Reg)
            regDegree=degree
        if type == 'reg':
            degree = regDegree
            self.prevGoalDegree=int(round((degree*0.293)-(self.default*0.293)))

        if read == 1:
            prev = self.read(self.ADDR_AX_PRES_POS,2)
            self.presentpos = prev[0]
        else:
            self.presentpos = self.prevGoal

        if (self.presentpos - regDegree) > 0 :
            self.speed_dir = -1
        else:
            self.speed_dir = 1

        difference = (abs(self.presentpos - regDegree)) * 0.293
     
        if time_type == 'sec':
            self.speed = difference/(time*0.666)
          
        elif time_type == 'rpm':
            self.speed = time/0.111
        
        elif time_type == 'omega':
            rpm=(time/(2*pi))*60
            self.speed=rpm/0.111
            # print('id: ', self.id, 'speed: ', self.speed*0.111, 'type: RPM')
        # else:
        #     self.speed = time #register

        self.prevGoal = degree
        
        self.speed = int(self.speed) + 5
        if self.speed >= 1023:
            speed = 1023
            print("id %s reach maxspeed" %self.id)
        if self.speed < 7 :
            speed = 0
        # print("id: %s degreenow: %s goal: %s speed : %s diff(degree): %.2f diff(reg): %s"%(self.id,prespos[0],regDegree,int(speed),difference,abs(prespos[0] - regDegree)))
        
        self.param = [DXL_LOBYTE(DXL_LOWORD(degree)),\
                      DXL_HIBYTE(DXL_LOWORD(degree)), \
                      DXL_LOBYTE(DXL_LOWORD(int(self.speed))), \
                      DXL_HIBYTE(DXL_LOWORD(int(self.speed)))]
        # print("id     : ", self.id, '\t', "degree : ", hex(degree), '\t',"param  : ", self.param)
    
