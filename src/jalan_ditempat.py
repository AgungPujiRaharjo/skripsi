#! /usr/bin/python3

import rospy
import pandas as pd
from openpyxl import Workbook
from geometry_msgs.msg import Vector3

from inverse_kinematic import *
from servo_controller import *

def bezier_curve_2D(phase, p1, p2):
    
    x = (1-phase) * p1[0,0] + phase * p2[0,0]
    y = (1-phase) * p1[0,1] + phase * p2[0,1]
    z = (1-phase) * p1[0,2] + phase * p2[0,2]

    return np.matrix([x,y,z])

def bezier_curve_3D(phase, p1, p2, p3):
    
    x = ((1 - phase)**2 * p1[0,0]) + (2 * (1 - phase) * phase * p2[0,0]) + (phase**2 * p3[0,0])
    y = ((1 - phase)**2 * p1[0,1]) + (2 * (1 - phase) * phase * p2[0,1]) + (phase**2 * p3[0,1])
    z = ((1 - phase)**2 * p1[0,2]) + (2 * (1 - phase) * phase * p2[0,2]) + (phase**2 * p3[0,2])

    return np.matrix([x,y,z])

def bezier_curve_4D(phase, p1, p2, p3, p4):
    
    x = ((1 - phase)**3 * p1[0,0]) + (3 * (1 - phase)**2 * phase * p2[0,0]) + (3 * (1-phase) * (phase**2) * p3[0,0]) + (phase**3 * p4[0,0])
    y = ((1 - phase)**3 * p1[0,1]) + (3 * (1 - phase)**2 * phase * p2[0,1]) + (3 * (1-phase) * (phase**2) * p3[0,1]) + (phase**3 * p4[0,1])
    z = ((1 - phase)**3 * p1[0,2]) + (3 * (1 - phase)**2 * phase * p2[0,2]) + (3 * (1-phase) * (phase**2) * p3[0,2]) + (phase**3 * p4[0,2])

    return np.matrix([x,y,z])

def bezier_curve_6D(phase, p1, p2, p3, p4, p5, p6):

    x = ((1 - phase)**5 * p1[0,0]) + (5 * (1 - phase)**4 * phase * p2[0,0]) + (10 * (1 - phase)**3 * (phase**2) * p3[0,0]) + (10 * (1 - phase)**2 * (phase**3) * p4[0,0]) + (5 * (1 - phase) * (phase**4) * p5[0,0]) + ((phase**5) * p6[0,0])
    y = ((1 - phase)**5 * p1[0,1]) + (5 * (1 - phase)**4 * phase * p2[0,1]) + (10 * (1 - phase)**3 * (phase**2) * p3[0,1]) + (10 * (1 - phase)**2 * (phase**3) * p4[0,1]) + (5 * (1 - phase) * (phase**4) * p5[0,1]) + ((phase**5) * p6[0,1])
    z = ((1 - phase)**5 * p1[0,2]) + (5 * (1 - phase)**4 * phase * p2[0,2]) + (10 * (1 - phase)**3 * (phase**2) * p3[0,2]) + (10 * (1 - phase)**2 * (phase**3) * p4[0,2]) + (5 * (1 - phase) * (phase**4) * p5[0,2]) + ((phase**5) * p6[0,2])

    return np.matrix([x,y,z])

def bezier_curve_8D(phase, p1, p2, p3, p4, p5, p6, p7, p8):

    x = ((1 - phase)**7 * p1[0,0]) + (7 * (1 - phase)**6 * phase * p2[0,0]) + (21 * (1 - phase)**5 * (phase**2) * p3[0,0]) + (35 * (1 - phase)**4 * (phase**3) * p4[0,0]) + (35 * (1 - phase)**3 * (phase**4) * p5[0,0]) + (21 * (1 - phase)**2 * (phase**5) * p6[0,0]) + (7 * (1 - phase) * (phase**6) * p7[0,0]) + ((phase**7) * p8[0,0])
    y = ((1 - phase)**7 * p1[0,1]) + (7 * (1 - phase)**6 * phase * p2[0,1]) + (21 * (1 - phase)**5 * (phase**2) * p3[0,1]) + (35 * (1 - phase)**4 * (phase**3) * p4[0,1]) + (35 * (1 - phase)**3 * (phase**4) * p5[0,1]) + (21 * (1 - phase)**2 * (phase**5) * p6[0,1]) + (7 * (1 - phase) * (phase**6) * p7[0,1]) + ((phase**7) * p8[0,1])
    z = ((1 - phase)**7 * p1[0,2]) + (7 * (1 - phase)**6 * phase * p2[0,2]) + (21 * (1 - phase)**5 * (phase**2) * p3[0,2]) + (35 * (1 - phase)**4 * (phase**3) * p4[0,2]) + (35 * (1 - phase)**3 * (phase**4) * p5[0,2]) + (21 * (1 - phase)**2 * (phase**5) * p6[0,2]) + (7 * (1 - phase) * (phase**6) * p7[0,2]) + ((phase**7) * p8[0,2])

    return np.matrix([x,y,z])

com_data = np.array([.0, .0, .0])
zmp_data = np.array([.0, .0, .0])
acc_data = np.array([.0, .0, .0])

igl_data = 0.0

def com_callback(msg):
    com_data[0] = msg.x
    com_data[1] = msg.y
    com_data[2] = msg.z

def zmp_callback(msg):
    zmp_data[0] = msg.x
    zmp_data[1] = msg.y
    zmp_data[2] = msg.z

def acc_callback(msg):
    acc_data[0] = msg.x
    acc_data[1] = msg.y
    acc_data[2] = msg.z

igl_data_roll = 0.0

def main():
    global igl_data_roll
    rospy.init_node('jalan_ditempat', anonymous=False)
    rospy.Subscriber("com", Vector3, com_callback)
    rospy.Subscriber("zmp", Vector3, zmp_callback)
    rospy.Subscriber("acc", Vector3, acc_callback)
    # com_pub = rospy.Publisher('com_data', Vector3, queue_size=1)
    rate = rospy.Rate(30)

    # com_msg = Vector3()

    ik = InverseKinematic()
    sc = ServoController()
    
    
    rows = []
    i = 0
    AXIS  = np.array([0,-1,1,-1,-1,1, 0,-1,-1,1,1,1, 0,0,0, 0,0,0])
    COM   = np.matrix([0.0, 0.0, 0.235])
    LEFT  = np.matrix([0.0, 58 / 1000, 0.0])
    RIGHT = np.matrix([0.0, -58 / 1000, 0.0])
    phase = 0

    uCOM   = np.matrix([0.0, 0.0, 0.235])
    uLEFT  = np.matrix([0.0, 58 / 1000, 0.0])
    uRIGHT = np.matrix([0.0, -58 / 1000, 0.0])
    
    time_start = rospy.Time.now().to_sec()
    phase = 0.0
    state_time = np.array([5,2,4,4,4,2,1])
    time = rospy.Time.now().to_sec()
    state = 0

    vel_sebelumnya = 0
    acc_sebelumnya = 0

    while not rospy.is_shutdown():
        if phase >= 1:
            time_start = rospy.Time.now().to_sec()
            phase = 0

            if state == 6:
                break; 
            else :
                state = state + 1

            uCOM = COM
            uRIGHT = RIGHT
            uLEFT = LEFT

        else:
            time_now = rospy.Time.now().to_sec() - time_start
            phase = time_now / state_time[state]
 
        delta_pitch1b = 0
        delta_pitch2b = 0

        if state == 0:
            print("WAIT")        
        if state == 1:
            print("state 1")
            delta_pitch1b = 0.04
            COM = bezier_curve_6D(phase, uCOM, np.matrix([0.0, 0.03, 0.235]), np.matrix([0.0, 0.08, 0.235]), np.matrix([0.01, 0.08, 0.235]), np.matrix([0.01, 0.08, 0.235]), np.matrix([0.01, 0.08, 0.235]))
            RIGHT = bezier_curve_3D(phase, uRIGHT, np.matrix([0.0, -0.058, 0.03]), np.matrix([0.0, -0.07, 0.1]))
            LEFT = bezier_curve_2D(phase, uLEFT,  np.matrix([0.0, 0.058, 0.0]))
            # ik.TILT = 12
        elif state == 2:
           print("state 2")
           delta_pitch2b = 0.035
           COM = bezier_curve_8D(phase, uCOM, np.matrix([0.01, 0.07, 0.235]), np.matrix([0.01, 0.04, 0.235]), np.matrix([0.0, -0.04, 0.235]),  np.matrix([0.01, -0.06, 0.235]),  np.matrix([0.01, -0.075, 0.235]), np.matrix([0.01, -0.075, 0.235]), np.matrix([0.01, -0.075, 0.235]))
           RIGHT = bezier_curve_8D(phase, uRIGHT, np.matrix([-0.01, -0.058, 0.05]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]))
           LEFT = bezier_curve_8D(phase, uLEFT,  np.matrix([0.0, 0.058, 0.0]), np.matrix([0.0, 0.058, 0.0]), np.matrix([0.0, 0.058, 0.0]), np.matrix([0.0, 0.058, 0.0]), np.matrix([0.0, 0.058, 0.05]), np.matrix([0.0, 0.058, 0.08]), np.matrix([0.0, 0.07, 0.1]))
        elif state == 3:
           print("state 3")
           delta_pitch1b = 0.03
           COM = bezier_curve_8D(phase, uCOM, np.matrix([0.01, -0.07, 0.235]), np.matrix([0.01, -0.04, 0.235]), np.matrix([0.0, 0.04, 0.235]),  np.matrix([0.01, 0.06, 0.235]),  np.matrix([0.01, 0.08, 0.235]), np.matrix([0.01, 0.08, 0.235]), np.matrix([0.01, 0.08, 0.235]))
           RIGHT = bezier_curve_8D(phase, uRIGHT, np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.05]), np.matrix([-0.01, -0.058, 0.08]), np.matrix([-0.01, -0.058, 0.1]))
           LEFT = bezier_curve_8D(phase, uLEFT,  np.matrix([-0.01, 0.058, 0.05]), np.matrix([-0.01, 0.058, 0.0]), np.matrix([-0.01, 0.058, 0.0]), np.matrix([-0.01, 0.058, 0.0]), np.matrix([-0.01, 0.058, 0.0]), np.matrix([-0.01, 0.058, 0.0]), np.matrix([-0.01, 0.058, 0.0]))
        elif state == 4:
           print("state 4")
           delta_pitch2b = 0.035
           COM = bezier_curve_8D(phase, uCOM, np.matrix([0.01, 0.07, 0.235]), np.matrix([0.01, 0.04, 0.235]), np.matrix([0.0, -0.04, 0.235]),  np.matrix([0.01, -0.06, 0.235]),  np.matrix([0.01, -0.075, 0.235]), np.matrix([0.01, -0.075, 0.235]), np.matrix([0.01, -0.075, 0.235]))
           RIGHT = bezier_curve_8D(phase, uRIGHT, np.matrix([-0.01, -0.058, 0.05]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]), np.matrix([-0.01, -0.058, 0.0]))
           LEFT = bezier_curve_8D(phase, uLEFT,  np.matrix([0.0, 0.058, 0.0]), np.matrix([0.0, 0.058, 0.0]), np.matrix([0.0, 0.058, 0.0]), np.matrix([0.0, 0.058, 0.0]), np.matrix([0.0, 0.058, 0.05]), np.matrix([0.0, 0.058, 0.08]), np.matrix([0.0, 0.07, 0.1]))
        elif state == 5:
           print("state 5")
           COM = bezier_curve_3D(phase, uCOM, np.matrix([0.01, -0.03, 0.235]), np.matrix([0.0, -0.0, 0.235]))
           RIGHT = bezier_curve_2D(phase, uRIGHT, np.matrix([0.0, -0.058, 0.0]))
           LEFT = bezier_curve_3D(phase, uLEFT, np.matrix([-0.01, 0.07, 0.0]), np.matrix([-0.01, 0.058, 0.0]))
           ik.TILT = 10
        elif state == 6:
            print("state 6")

        JOINTS = ik.solve(COM,LEFT, RIGHT)

        K = np.array([1, 1, 1])
        delta = COM[0,1]*K[0] + (acc_data[1]*(time-rospy.Time.now().to_sec())*K[1])+vel_sebelumnya + acc_data[1]*K[2]
        
        acc_robot = delta*(time-rospy.Time.now().to_sec())+acc_sebelumnya
        vel_robot = acc_robot*(time-rospy.Time.now().to_sec())+vel_sebelumnya
        pos_robot = vel_robot*(time-rospy.Time.now().to_sec())
        COM[0,1] += pos_robot 

        time = rospy.Time.now().to_sec()
        JOINTS[0] = 372
        JOINTS[6] = 674
        TANGAN = [821,624,592, 207,400,435]

        JOINTS = np.array(np.hstack((JOINTS,TANGAN)))

        sc.sync_write_pos(JOINTS*AXIS)

        acc_sebelumnya = acc_data[1]
        vel_sebelumnya = acc_sebelumnya*(time-rospy.Time.now().to_sec())

        rows.append([rospy.Time.now().to_sec(), com_data[0], com_data[1], com_data[2], zmp_data[0], zmp_data[1]])

        # com_msg.x = COM[0,0]
        # com_msg.y = COM[0,1]
        # com_msg.z = COM[0,2]

        # com_pub.publish(com_msg)

        print(COM[0,0], COM[0,1], COM[0,2])
        
        rate.sleep()
    df = pd.DataFrame(rows, columns=['time','COM_X', 'COM_Y', 'COM_Z', 'ZMP_X', 'ZMP_Y'])
    # Create a Pandas Excel writer using XlsxWriter as the engine.
    writer = pd.ExcelWriter('demo.xlsx', engine='xlsxwriter')

    # Convert the dataframe to an XlsxWriter Excel object.
    df.to_excel(writer, sheet_name='Sheet1', index=False)
    writer.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
