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

"""
Sensors touch and force 

              ## Head Touch
              "Device/SubDeviceList/Head/Touch/Front/Sensor/Value",  # 0
              "Device/SubDeviceList/Head/Touch/Rear/Sensor/Value",   # 1
              "Device/SubDeviceList/Head/Touch/Middle/Sensor/Value", #2
              ## LeftHandTouch
              "Device/SubDeviceList/LHand/Touch/Back/Sensor/Value", #3 
              "Device/SubDeviceList/LHand/Touch/Left/Sensor/Value", #4 
              "Device/SubDeviceList/LHand/Touch/Right/Sensor/Value", #5
              ## RightHandTouch 
              "Device/SubDeviceList/RHand/Touch/Back/Sensor/Value", #6
              "Device/SubDeviceList/RHand/Touch/Left/Sensor/Value", #7 
              "Device/SubDeviceList/RHand/Touch/Right/Sensor/Value" #8 
              ## LeftFSR 
              "Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value", #9
              "Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value", #10 
              "Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value",  #11 
              "Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value",  #12
              ## RightFSR 
              "Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value", #13 
              "Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value", #14
              "Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value",  #15
              "Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value", #16 
              ## Body Angle
              "Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value", #17 
              "Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value"  #18 
"""
import naoqi
from naoqi import ALProxy
import math
import time
from naoqi import ALModule
#import matplotlib.pyplot as plt
import numpy as np
from SetTiming import *
from MLMPCPG import *
from NAOMotor import *
from random import randint
from myPloting import * 
######################################
#NAOIP = "Orango.local"
#NAOIP = "nao.local"
#NAOIP = "nao.local"

#NAOIP = "0.0.0.0"
NAOIP = "nao.local"

PORT= 9559 ; 

number_cpg = 26 ;

global All_Command 
global All_Sensor


global myCont,CurPos,myT


All_Command=[]
All_Sensor=[]

######################################
# Connect to the module ALMotionProxy 
movObj = ALProxy("ALMotion",NAOIP,PORT)
movObj.setStiffnesses('Body',0.99)


# Connect to the module ALRobotPostureProxy
postObj = ALProxy("ALRobotPosture",NAOIP,PORT)

# Connect to the module ALMemoryProxy
memProxy = ALProxy("ALMemory",NAOIP,PORT)


#############


fractionMaxSpeed = 1.0

# Disable Fall Manager 
# movObj.setFallManagerEnabled(False) # True False
# time.sleep(1)

# http://doc.aldebaran.com/2-1/family/robots/postures_robot.html#robot-postures
postObj.goToPosture("Crouch",0.8)
time.sleep(2)

#StandInit
#StandZero
#Crouch

myT = fSetTiming()

myCont = fnewMLMPcpg(number_cpg)

myCont = fSetCPGNet(myCont,'MyNao.txt','MyNaoPsitiveAngle_E_or_F.txt')
#myCont2[25].RG.E.Es = 99



PatternOsc = RG_Patterns(17.5,17.5,1,0.05)
#PatternOsc1.fPrintPattern()

"""
  PL
   ^
   |     ------------
   |   /
   |  / 
   | /
   ------------------------> 
 """
PatternPL1 = RG_Patterns(5,0.1,1,0.1)
#PatternPL1.fPrintPattern()

"""
  QUIESCENT
   ^
   |     
   |   /\
   |  /  \
   | /    \____________
   ------------------------> 
 """
PatternQU = RG_Patterns(0.5,0.5,5,0.1)
#PatternQU.fPrintPattern()

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
PatternAosc = RG_Patterns(0.9,5,1,0.1)
#PatternAosc.fPrint()


#LSP_PFPattern = PF_Patterns(0.1,0)
PFPat1 = PF_Patterns(0.05,0)
PFPat2 = PF_Patterns(0.05,0)


myCont[L_SHOULDER_PITCH].fSetPatternPF(PFPat1)
myCont[L_SHOULDER_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[L_ELBOW_ROLL].fSetPatternPF(PFPat2)
myCont[L_ELBOW_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU


myCont[L_SHOULDER_PITCH].fPrint()
myCont[L_ELBOW_ROLL].fPrint()

"""
HEAD_YAW,HEAD_PITCH,L_SHOULDER_PITCH,L_SHOULDER_ROLL,L_ELBOW_YAW,L_ELBOW_ROLL,
L_WRIST_YAW,L_HAND,L_HIP_YAW_PITCH,L_HIP_ROLL,L_HIP_PITCH,L_KNEE_PITCH,L_ANKLE_PITCH,
L_ANKLE_ROLL,R_HIP_YAW_PITCH,R_HIP_ROLL,R_HIP_PITCH,R_KNEE_PITCH,R_ANKLE_PITCH,
R_ANKLE_ROLL,R_SHOULDER_PITCH,R_SHOULDER_ROLL,R_ELBOW_YAW,R_ELBOW_ROLL,
R_WRIST_YAW,R_HAND
"""

# Prepare the robot to start the CPG by going into a starting position 
movObj.setAngles('LShoulderPitch',0, fractionMaxSpeed)
movObj.setAngles('LElbowRoll',-45*math.pi/180.0, fractionMaxSpeed)

time.sleep(4)

CurPos = movObj.getAngles('Body',True)

for i in range(0, len(myCont)):
    myCont[i].fUpdateInitPos(CurPos[i])


#############################
for i in range(0, len(myCont)):
    myCont[i].fUpdateLocomotionNetwork(myT,CurPos[i])
#############################

######################################
# "MiddleTactileON" CLOSE THE HAND TO PICK THE BALL 
while True:
    MiddleTactileON = memProxy.getData('Device/SubDeviceList/Head/Touch/Middle/Sensor/Value')
    if (MiddleTactileON):
        movObj.setAngles('LHand', 0, 0.5)
        time.sleep(1)
        break
######################################



time1 = time.time()

MAT_Iinj=[];
for I in range(0,int(myT.N_Loop/2)):
    t= I*myT.T;

    # #################
    # ExtInjCurr
    # #################
    if t>= myT.T1 and t < myT.T2:
        ExtInjCurr = 1
    else: 
        ExtInjCurr = 0
        
    if t>= myT.T3 and t < myT.T4:
        ExtInjCurr2 = 1
    else: 
        ExtInjCurr2 = 0

    MAT_Iinj.append(ExtInjCurr) 

    #L_SHOULDER_PITCH ,
    for ii in [ L_SHOULDER_PITCH, L_ELBOW_ROLL]:
        myCont[ii].RG.F.InjCurrent_value = +1*ExtInjCurr* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value =  -1*ExtInjCurr* myCont[ii].RG.E.InjCurrent_MultiplicationFactor

  
    #for ii in [L_ELBOW_ROLL]:
    #    myCont[ii].RG.F.InjCurrent_value = +1*ExtInjCurr2* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
    #    myCont[ii].RG.E.InjCurrent_value = -1*ExtInjCurr2* myCont[ii].RG.E.InjCurrent_MultiplicationFactor

    
    for i in [L_SHOULDER_PITCH, L_ELBOW_ROLL]:
        myCont[i].fUpdateLocomotionNetwork(myT,CurPos[i])

        
    for i in range(0, len(myCont)):
        MotorCommand[i]=myCont[i].joint.joint_motor_signal
    
    movObj.setAngles('Body',MotorCommand , fractionMaxSpeed)
    ##movObj.angleInterpolation('Body',MotorCommand , 0.03, True)
    

     # Read motor angel from robot joints 
    CurPos = movObj.getAngles('Body',True)
    
    All_Command.append(MotorCommand[:]) 
    All_Sensor.append(CurPos)
    
    #data2print1.append(myCont[L_HIP_ROLL].SN.E.o)
    #data2print2.append(myCont[L_HIP_ROLL].SN.F.o)
    
    ######################################
    
    
    time2 = time.time()
    while (time2 - time1)<0.04 :
        time2 = time.time()
    print 'time diff: ', time2 - time1
    #time_diff  = time2 - time1
    time1 = time2 
    

    RearTactileON = memProxy.getData('Device/SubDeviceList/Head/Touch/Rear/Sensor/Value')
    if (RearTactileON):
        print 'Walking has been stoped by toutching the rear tactile head sensor.\n'
        break
        

#print 'time diff: ', time_diff
##############################

postObj.goToPosture("Crouch",0.8)
time.sleep(2)
movObj.setStiffnesses('Body',0.0)


# Print motion data ..
##############################
f = open('AllCommands', 'w')
s = str(All_Command)
f.write(s)
f.close()

f = open('AllAngels', 'w')
s = str(All_Sensor)
f.write(s)
f.close()

##############################

