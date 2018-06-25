# -*- coding: utf-8 -*-
"""
Created on Sun Jun 11 15:59:26 2017

@author: nasjo
"""

import sys
sys.path.append('../../mylib/MLMPCPG')

import matplotlib.pyplot as plt
import math
import time
import numpy as np

from NAOMotor import *

def fPlot2DLeftArm(All_Command,All_Sensor,StartInd,Str,ArmDraw):
    

    # np.matrix( ((1,2), (5, -1)) )
    
    # Main length           (mm)
    ShoulderOffsetY       =98.00
    ElbowOffsetY          =15.00
    UpperArmLength        =105.00
    LowerArmLength        =55.95
    ShoulderOffsetZ       =100.00
    HandOffsetX           =57.75
    HandOffsetZ           =12.31
    
    # parameters used in the transformation below
    a2 = ElbowOffsetY
    y0 = ShoulderOffsetY
    z0 = ShoulderOffsetZ
    d3 = UpperArmLength
    ye = HandOffsetZ
    ze = LowerArmLength + HandOffsetX
    
    """
    3. LShoulderPitch 	Left shoulder joint front and back (Y) 	-119.5 to 119.5 	-2.0857 to 2.0857
    4. LShoulderRoll 	Left shoulder joint right and left (Z) 	-18 to 76           -0.3142 to 1.3265
    5. LElbowYaw 	Left shoulder joint twist (X)               -119.5 to 119.5 	-2.0857 to 2.0857
    6. LElbowRoll 	Left elbow joint (Z)                        -88.5 to -2         -1.5446 to -0.0349
    7. LWristYaw 	Left wrist joint (X)                        -104.5 to 104.5 	-1.8238 to 1.8238
    8. LHand         Left hand 	Open and Close 	Open and Close
    """

    theta1=[]
    for i in range(0,len(All_Sensor)-StartInd):
        theta1.append(All_Sensor[i+StartInd][L_SHOULDER_PITCH])
    
    
    theta2=[]
    for i in range(0,len(All_Sensor)-StartInd):
        theta2.append(All_Sensor[i+StartInd][L_SHOULDER_ROLL])

    theta3=[]
    for i in range(0,len(All_Sensor)-StartInd):
        theta3.append(All_Sensor[i+StartInd][L_ELBOW_YAW])

    theta4=[]
    for i in range(0,len(All_Sensor)-StartInd):
        theta4.append(All_Sensor[i+StartInd][L_ELBOW_ROLL])

    theta5=[]
    for i in range(0,len(All_Sensor)-StartInd):
        theta5.append(All_Sensor[i+StartInd][L_WRIST_YAW])
    
    TrajX=[]
    TrajY=[]
    
    fig = plt.figure(figsize=(8,6.73))

    for i in range(0,len(theta1)):
        t1 = theta1[i]
        t2 = theta2[i]
        t3 = theta3[i]
        t4 = theta4[i]
        t5 = theta5[i]

        #  translation from the base-frame to frame-0
        A0 = np.matrix( ((1,    0,   0,   0), (0,    0,   1,   y0),(0,   -1,   0,   z0),(0,    0,   0,   1)) )
        # translation from-0 to frame-1
        A1=np.matrix( (( math.cos(t1),0,math.sin(t1),0),(math.sin(t1),0,-math.cos(t1),0),(0,1,0,0),(0,0,0,1)) )
        # translation from-1 to frame-2
        A2=np.matrix( (( -math.sin(t2), 0, math.cos(t2), -a2*math.sin(t2)),(math.cos(t2),0,math.sin(t2),a2*math.cos(t2)),(0,1,0,0),(0,0,0,1)) )
        # translation from-2 to frame-3
        A3=np.matrix( (( math.cos(t3),0,-math.sin(t3),0),(math.sin(t3),0,math.cos(t3),0),(0,-1,0,d3),(0,0,0,1)) )
        # translation from-3 to frame-4  
        A4=np.matrix( ((math.cos(t4),0,math.sin(t4),0),(math.sin(t4),0,-math.cos(t4),0),(0,1,0,0),(0,0,0,1)) )
        # translation from-4 to frame-5  
        A5=np.matrix( ((-math.sin(t5),-math.cos(t5),0,-ye*math.sin(t5)),(math.cos(t5),-math.sin(t5),0,ye*math.cos(t5)),(0,0,1,ze),(0,0,0,1)) )
        
        TrElb = np.mat(A0) * np.mat(A1) * np.mat(A2) * np.mat(A3)
        TrEnd = np.mat(A0) * np.mat(A1) * np.mat(A2) * np.mat(A3) * np.mat(A4) * np.mat(A5)
        #print Tr
        #print 'len(A0)= ',len(A0)
        #print 'len(TrElb)= ',len(TrElb)
        #print 'len(TrEnd)= ',len(TrEnd)
        
        XX=-1*np.array ([A0[1,3] ,TrElb[1,3] ,TrEnd[1,3]])
        YY=np.array ([A0[0,3] ,TrElb[0,3] ,TrEnd[0,3]])
        

        if ArmDraw=='arm_on':
            plt.plot( XX, YY,'g')
        plt.grid(True)
        plt.hold(True)
        plt.axis([-350,50,0,350])
        

        TrajX.append(-TrEnd[1,3])
        TrajY.append(TrEnd[0,3])
        
 
    plt.plot( TrajX, TrajY,'b',lw=3)
    SW = np.loadtxt('NAOROBOT\workspace2DLArm4Drawing.out') 
    plt.plot(SW[0],SW[1],'g',lw=1)


    #plt.show(block=False)
    plt.show()
    
    return TrajX , TrajY;

