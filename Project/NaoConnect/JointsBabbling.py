# -*- coding: utf-8 -*-
"""
Created on Sun May 28 21:02:20 2017

@author: nasjo
"""

"""
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Joint name 	Motion                                          Range (degrees) 	Range (radians)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  1. HeadYaw       Head joint twist (Z)                        -119.5 to 119.5 	-2.0857 to 2.0857
  2. HeadPitch 	Head joint front and back (Y)               -38.5 to 29.5       -0.6720 to 0.5149

  3. LShoulderPitch 	Left shoulder joint front and back (Y) 	-119.5 to 119.5 	-2.0857 to 2.0857
  4. LShoulderRoll 	Left shoulder joint right and left (Z) 	-18 to 76           -0.3142 to 1.3265
  5. LElbowYaw 	Left shoulder joint twist (X)               -119.5 to 119.5 	-2.0857 to 2.0857
  6. LElbowRoll 	Left elbow joint (Z)                        -88.5 to -2         -1.5446 to -0.0349
  7. LWristYaw 	Left wrist joint (X)                        -104.5 to 104.5 	-1.8238 to 1.8238
  8. LHand         Left hand 	Open and Close 	Open and Close

  9. LHipYawPitch* 	Left hip joint twist (Y-Z 45deg)          -65.62 to 42.44 	-1.145303 to 0.740810
 10. LHipRoll      Left hip joint right and left (X)           -21.74 to 45.29 	-0.379472 to 0.790477
 11. LHipPitch 	Left hip joint front and back (Y)           -88.00 to 27.73 	-1.535889 to 0.484090
 12. LKneePitch 	Left knee joint (Y)                         -5.29 to 121.04 	-0.092346 to 2.112528
 13. LAnklePitch 	Left ankle joint front and back (Y)         -68.15 to 52.86 	-1.189516 to 0.922747
 14. LAnkleRoll 	Left ankle joint right and left (X)         -22.79 to 44.06 	-0.397880 to 0.769001

 15. RHipYawPitch* 	Right hip joint twist (Y-Z 45deg)         -65.62 to 42.44 	-1.145303 to 0.740810
 16. RHipRoll      Right hip joint right and left (X)          -45.29 to 21.74 	-0.790477 to 0.379472
 17. RHipPitch 	Right hip joint front and back (Y)          -88.00 to 27.73 	-1.535889 to 0.484090
 18. RKneePitch 	Right knee joint (Y)                        -5.90 to 121.47 	-0.103083 to 2.120198
 19. RAnklePitch 	Right ankle joint front and back (Y)        -67.97 to 53.40 	-1.186448 to 0.932056
 20. RAnkleRoll 	Right ankle joint right and left (X)        -44.06 to 22.80 	-0.768992 to 0.397935

 21. RShoulderPitch 	Right shoulder joint front and back (Y)	-119.5 to 119.5 	-2.0857 to 2.0857
 22. RShoulderRoll 	Right shoulder joint right and left (Z)	-76 to 18           -1.3265 to 0.3142
 23. RElbowYaw 	Right shoulder joint twist (X)              -119.5 to 119.5 	-2.0857 to 2.0857
 24. RElbowRoll 	Right elbow joint (Z)                       2 to 88.5           0.0349 to 1.5446
 25. RWristYaw 	Right wrist joint (X)                       -104.5 to 104.5 	-1.8238 to 1.8238
 26. RHand         Right hand 	Open and Close 	Open and Close

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% REF for postures
% http://doc.aldebaran.com/1-14/naoqi/motion/alrobotposture.html#term-predefined-postures
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
"""

import sys
#sys.path.append('lib/NaoConnect')
#sys.path.append('lib/CPG')

import NaoConnect
import time
import numpy as np
import vrep

######################################

from SetTiming import *
from MLMPCPG import *
from NAOMotor import *
from random import randint
#from myPloting import *

######################################

if NaoConnect.NaoRobotConnect.RealNaoRobot:
    print "RealNaoRobot: ", NaoConnect.NaoRobotConnect.RealNaoRobot[0]

if NaoConnect.NaoVrepConnect.NaoVrep:
    print "NaoVrep: ", NaoConnect.NaoVrepConnect.NaoVrep[0]

NAOosON = NaoConnect.NaoRobotConnect.RealNaoRobot or NaoConnect.NaoVrepConnect.NaoVrep

if NAOosON == []:
    sys.exit("No robot or simulation connected..!")

print "NAOosON : ", NAOosON

#######################################################

number_cpg = 26

global All_Command

global All_Joints_Sensor

global myCont, angles, myT, Good_pose, CurPos

All_Command = []
All_Joints_Sensor = []

Good_pose = 0
CurPos = []

fractionMaxSpeed = 0.3

#######################################################
myT = fSetTiming()

# Creat list of CPG objects
myCont = fnewMLMPcpg(number_cpg)

# Inisiate the CPG list with NAO robot data
myCont = fSetCPGNet(myCont, 'MyNao.txt', 'MyNaoPsitiveAngle_E_or_F.txt')
#######################################################

PatternOsc = RG_Patterns(17.5, 17.5, 1, 0.05)
# PatternOsc1.fPrintPattern()

"""
  PL
   ^
   |     ------------
   |   /
   |  / 
   | /
   ------------------------> 
 """
PatternPL1 = RG_Patterns(5, 0.1, 1, 0.1)
# PatternPL1.fPrintPattern()

"""
  QUIESCENT
   ^
   |     
   |   /\
   |  /  \
   | /    \____________
   ------------------------> 
 """
PatternQU = RG_Patterns(0.5, 0.5, 5, 0.1)
# PatternQU.fPrintPattern()

"""
  Almost-Osc
   ^
   |     
   |   /\
   |  /  \    /\  
   | /    \  /  \/\-----
   |/      \/
   ------------------------> 
 """
PatternAosc = RG_Patterns(0.9, 5, 1, 0.1)
# PatternAosc.fPrint()

#######################################################
# Joint Index to be used when referring to a particular joint

"""
HEAD_YAW,HEAD_PITCH,L_SHOULDER_PITCH,L_SHOULDER_ROLL,L_ELBOW_YAW,L_ELBOW_ROLL,
L_WRIST_YAW,L_HAND,L_HIP_YAW_PITCH,L_HIP_ROLL,L_HIP_PITCH,L_KNEE_PITCH,L_ANKLE_PITCH,
L_ANKLE_ROLL,R_HIP_YAW_PITCH,R_HIP_ROLL,R_HIP_PITCH,R_KNEE_PITCH,R_ANKLE_PITCH,
R_ANKLE_ROLL,R_SHOULDER_PITCH,R_SHOULDER_ROLL,R_ELBOW_YAW,R_ELBOW_ROLL,
R_WRIST_YAW,R_HAND
"""
######################################
# read angles for update CPG

angles = NaoConnect.NaoGetAngles()

CurPos = angles

#######################################################
# Update CPG initial position (reference position)

for i in range(0, len(myCont)):
    myCont[i].fUpdateInitPos(angles[i])

#############################
# Update all joints CPG, it is important to update all joints
# at least one time, otherwise, non used joints will be set to
# the defult init posiotion in the CPG which is 0

for i in range(0, len(myCont)):
    myCont[i].fUpdateLocomotionNetworkSN(angles[i])

# Update other sensor neurons..TODO

#
for i in range(0, len(myCont)):
    myCont[i].fUpdateLocomotionNetwork(myT, angles[i])
#############################

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print ('Connection to V-rep established')

# OutFile = open('RArm_Shouder_Pitch.txt', 'w')
# OutFile = open('NNs/RArm_Shouder_Roll.txt', 'w')
# OutFile = open('NNs/RArm_Elbow_Yaw.txt', 'w')
# OutFile = open('NNs/RArm_Elbow_Roll.txt', 'w')
OutFile = open('R_ANKLE_PITCH.txt', 'w')
CurPos_end = NaoConnect.NaoGetAngles()

# MAIN LOOP
if __name__ == "__main__":  # this is to check if we are importing
    while Good_pose < 2000:

        #  Read motor angel from robot joints
        CurPos_start = CurPos_end

        alpha_1 = 0
        alpha_2 = 0
        alpha_3 = 0
        alpha_4 = 0
        alpha_5 = 0

        # alpha_1 = np.random.uniform(-0.3, 0.3)
        # alpha_2 = np.random.uniform(-0.9, 0.9)
        # alpha_3 = np.random.uniform(-1, 1)
        alpha_4 = np.random.uniform(-0.9, 0.9)
        # alpha_5 = np.random.uniform(-0.4, 0.4)

        # print 'alpha:', alpha_1

        PFPattern1 = PF_Patterns(alpha_1, 0)
        PFPattern2 = PF_Patterns(alpha_2, 0)
        PFPattern3 = PF_Patterns(alpha_3, 0)
        PFPattern4 = PF_Patterns(alpha_4, 0)
        PFPattern5 = PF_Patterns(alpha_5, 0)

        myCont[R_HIP_ROLL].fSetPatternPF(PFPattern1)
        myCont[R_HIP_ROLL].fSetPatternRG(PatternPL1)  # PatternPL1 PatternOsc3 PatternAosc PatternQU

        myCont[R_HIP_PITCH].fSetPatternPF(PFPattern2)
        myCont[R_HIP_PITCH].fSetPatternRG(PatternPL1)  # PatternPL1 PatternOsc3 PatternAosc PatternQU

        myCont[R_KNEE_PITCH].fSetPatternPF(PFPattern3)
        myCont[R_KNEE_PITCH].fSetPatternRG(PatternPL1)  # PatternPL1 PatternOsc3 PatternAosc PatternQU

        myCont[R_ANKLE_PITCH].fSetPatternPF(PFPattern4)
        myCont[R_ANKLE_PITCH].fSetPatternRG(PatternPL1)  # PatternPL1 PatternOsc3 PatternAosc PatternQU

        myCont[R_ANKLE_ROLL].fSetPatternPF(PFPattern5)
        myCont[R_ANKLE_ROLL].fSetPatternRG(PatternPL1)  # PatternPL1 PatternOsc3 PatternAosc PatternQU

        time1 = time.time()

        for I in range(0, int(myT.N_Loop / 2)):
            t = I * myT.T

            # #################
            # ExtInjCurr
            # #################
            # if t >= myT.T1 and t < myT.T2:

            ExtInjCurr = 1

            for ii in [R_HIP_ROLL]:
                myCont[ii].RG.F.InjCurrent_value = +1 * ExtInjCurr * myCont[ii].RG.F.InjCurrent_MultiplicationFactor
                myCont[ii].RG.E.InjCurrent_value = -1 * ExtInjCurr * myCont[ii].RG.E.InjCurrent_MultiplicationFactor

            for ii in [R_HIP_PITCH]:
                myCont[ii].RG.F.InjCurrent_value = +1 * ExtInjCurr * myCont[ii].RG.F.InjCurrent_MultiplicationFactor
                myCont[ii].RG.E.InjCurrent_value = -1 * ExtInjCurr * myCont[ii].RG.E.InjCurrent_MultiplicationFactor

            for ii in [R_KNEE_PITCH]:
                myCont[ii].RG.F.InjCurrent_value = +1 * ExtInjCurr * myCont[ii].RG.F.InjCurrent_MultiplicationFactor
                myCont[ii].RG.E.InjCurrent_value = -1 * ExtInjCurr * myCont[ii].RG.E.InjCurrent_MultiplicationFactor

            for ii in [R_ANKLE_PITCH]:
                myCont[ii].RG.F.InjCurrent_value = +1 * ExtInjCurr * myCont[ii].RG.F.InjCurrent_MultiplicationFactor
                myCont[ii].RG.E.InjCurrent_value = -1 * ExtInjCurr * myCont[ii].RG.E.InjCurrent_MultiplicationFactor

            for ii in [R_WRIST_YAW]:
                myCont[ii].RG.F.InjCurrent_value = +1 * ExtInjCurr * myCont[ii].RG.F.InjCurrent_MultiplicationFactor
                myCont[ii].RG.E.InjCurrent_value = -1 * ExtInjCurr * myCont[ii].RG.E.InjCurrent_MultiplicationFactor


            for i in [R_HIP_ROLL, R_HIP_PITCH, R_KNEE_PITCH, R_ANKLE_PITCH, R_ANKLE_ROLL]:
                myCont[i].fUpdateLocomotionNetwork(myT, CurPos[i])

                # , R_SHOULDER_ROLL, R_ELBOW_YAW, R_ELBOW_ROLL, R_WRIST_YAW

            for i in range(0, len(myCont)):
                MotorCommand[i] = myCont[i].joint.joint_motor_signal

            NaoConnect.NaoSetAngles(MotorCommand)

            CurPos = NaoConnect.NaoGetAngles()

            ######################################

            time2 = time.time()
            while (time2 - time1) < 0.04:
                time2 = time.time()
            # print 'time diff: ', time2 - time1
            # time_diff  = time2 - time1
            time1 = time2

        # Read motor angel from robot joints
        CurPos_end = CurPos
        print CurPos_end[R_ANKLE_PITCH]

        for i in [R_HIP_ROLL, R_HIP_PITCH, R_KNEE_PITCH, R_ANKLE_PITCH, R_ANKLE_ROLL]:
            myCont[i].RG.F.V = 0
            myCont[i].RG.F.q = 0
            myCont[i].RG.E.V = 0
            myCont[i].RG.E.q = 0
            myCont[i].joint.init_motor_pos = CurPos_end[i]

        # if -0.72 < CurPos_end[R_HIP_ROLL] < 0.24:
        # if -1.64 < CurPos_end[R_HIP_PITCH] < 0.31:
        # if 0.006 < CurPos_end[R_KNEE_PITCH] < 2:
        # if -1.08 < CurPos_end[R_ANKLE_PITCH] < 0.9:
        if -0.76 < CurPos_end[R_ANKLE_ROLL] < 0.385:


            OutFile.write(str(CurPos_end[R_ANKLE_PITCH] - CurPos_start[R_ANKLE_PITCH]))
            OutFile.write(",")
            OutFile.write(str(alpha_4))
            OutFile.write('\n')
            Good_pose = Good_pose + 1

            print 'Good_pose', Good_pose

        else:

            print 'Bad_pose'

OutFile.close()

