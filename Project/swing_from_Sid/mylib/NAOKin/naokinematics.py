# -*- coding: utf-8 -*-
"""
Created on Wed Jun 21 18:31:40 2017

@author: nasjo
"""

    
import math 
import numpy as np 

def fFK2DLeftArm(theta):
    """
    This function calculate the forward kinematics for NAO left arm.
    
    @param ang: left arm joint angles [LShoulderPitch,LShoulderRoll,LElbowYaw,LElbowRoll,LWristYaw]
    @return: End-effector transformation matrix 4X4 

    """
    

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

    t1 = theta[0]
    t2 = theta[1]
    t3 = theta[2]
    t4 = theta[3]
    t5 = theta[4]

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
        
    return TrEnd;



##############################


def frwKin(theta):
    theta=np.array(theta);
    if (theta.size > 5):
        t1 = np.array(theta[:,0]);
        t2 = np.array(theta[:,1]);
        t3 = np.array(theta[:,2]);
        t4 = np.array(theta[:,3]);
        t5 = np.array(theta[:,4]);
    elif (theta.size == 5):
        t1 = np.array([theta[0]]);
        t2 = np.array([theta[1]]);
        t3 = np.array([theta[2]]);
        t4 = np.array([theta[3]]);
        t5 = np.array([theta[4]]);
    else:
        print "number of thetas for the joints is less than 5, 5 joints for the arm"
    # Main length           (mm)
    ShoulderOffsetY = 98.00
    ElbowOffsetY = 15.00
    UpperArmLength = 105.00
    LowerArmLength = 55.95
    ShoulderOffsetZ = 100.00
    HandOffsetX = 57.75
    HandOffsetZ = 12.31

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
    if (t1.size==t2.size==t3.size==t4.size==t5.size):
        XX=[];
        YY=[];
        for i in range(0, t1.size):
            #  translation from the base-frame to frame-0
            A0 = np.matrix(((1, 0, 0, 0), (0, 0, 1, y0), (0, -1, 0, z0), (0, 0, 0, 1)))
            # translation from-0 to frame-1
            A1 = np.matrix(
                ((math.cos(t1[i]), 0, math.sin(t1[i]), 0), (math.sin(t1[i]), 0, -math.cos(t1[i]), 0), (0, 1, 0, 0), (0, 0, 0, 1)))
            # translation from-1 to frame-2
            A2 = np.matrix(((-math.sin(t2[i]), 0, math.cos(t2[i]), -a2 * math.sin(t2[i])),
                            (math.cos(t2[i]), 0, math.sin(t2[i]), a2 * math.cos(t2[i])), (0, 1, 0, 0), (0, 0, 0, 1)))
            # translation from-2 to frame-3
            A3 = np.matrix(
                ((math.cos(t3[i]), 0, -math.sin(t3[i]), 0), (math.sin(t3[i]), 0, math.cos(t3[i]), 0), (0, -1, 0, d3), (0, 0, 0, 1)))
            # translation from-3 to frame-4
            A4 = np.matrix(
                ((math.cos(t4[i]), 0, math.sin(t4[i]), 0), (math.sin(t4[i]), 0, -math.cos(t4[i]), 0), (0, 1, 0, 0), (0, 0, 0, 1)))
            # translation from-4 to frame-5
            A5 = np.matrix(((-math.sin(t5[i]), -math.cos(t5[i]), 0, -ye * math.sin(t5[i])),
                            (math.cos(t5[i]), -math.sin(t5[i]), 0, ye * math.cos(t5[i])), (0, 0, 1, ze), (0, 0, 0, 1)))

            TrElb = np.mat(A0) * np.mat(A1) * np.mat(A2) * np.mat(A3)
            TrEnd = np.mat(A0) * np.mat(A1) * np.mat(A2) * np.mat(A3) * np.mat(A4) * np.mat(A5)
            # print Tr
            # print 'len(A0)= ',len(A0)
            # print 'len(TrElb)= ',len(TrElb)
            # print 'len(TrEnd)= ',len(TrEnd)

            temp_YY = np.array([A0[1, 3], TrElb[1, 3], TrEnd[1, 3]])
            temp_XX = np.array([A0[0, 3], TrElb[0, 3], TrEnd[0, 3]])
            XX = np.append(XX, temp_XX, axis=0)
            YY = np.append(YY, temp_YY, axis=0)
            # XX = np.concatenate((XX, temp_XX), axis=0)
            # YY = np.concatenate((YY, temp_YY), axis=0)
        return XX, YY
    else:
        print "size mismatch in theta (vector) for the joints"

# t=np.array([[0,90,45,-90,0],[0,0,45,-0.0,0],[0,90,0,-90,180]])*math.pi/180;
# print frwKin(t)
