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
# import naoqi
# from naoqi import ALProxy
# import math
# import time
# from naoqi import ALModule

#import matplotlib.pyplot as plt
import numpy as np
from SetTiming import *
from MLMPCPG import *
from NAOMotor import *
from random import randint

import NaoConnect
import time
import numpy as np
import vrep
#from myPloting import * 
######################################
#NAOIP = "Orango.local"
#NAOIP = "nao.local"
#NAOIP = "nao.local"


if NaoConnect.NaoRobotConnect.RealNaoRobot:
    print "RealNaoRobot: ", NaoConnect.NaoRobotConnect.RealNaoRobot[0]

if NaoConnect.NaoVrepConnect.NaoVrep:
    print "NaoVrep: ", NaoConnect.NaoVrepConnect.NaoVrep[0]

NAOosON = NaoConnect.NaoRobotConnect.RealNaoRobot or NaoConnect.NaoVrepConnect.NaoVrep

if NAOosON == []:
    sys.exit("No robot or simulation connected..!")

print "NAOosON : ", NAOosON

#NAOIP = "169.254.40.17"

PORT= 9559

number_cpg = 26

global All_Command 
global All_Sensor
global readOscFreq,readOscFreq2,readOscFreqOverTime
global readLFSRFreq,readLFSRFreq2,readLFSRFreqOverTime
global readRFSRFreq,readRFSRFreq2,readRFSRFreqOverTime

global LFsrFL,LFsrFR,LFsrBL,LFsrBR,RFsrFL,RFsrFR,RFsrBL,RFsrBR,LHandBackSensor,LHandLeftSensor,LHandRightSensor,RHandBackSensor,RHandLeftSensor,RHandRightSensor 

global myCont,CurPos,myT


All_Command=[]
All_Sensor=[]

# ######################################
# # Connect to the module ALMotionProxy 
# movObj = ALProxy("ALMotion",NAOIP,PORT)
# movObj.setStiffnesses('Body',0.99)

# # Connect to the module ALTextToSpeechProxy
# TextObj = ALProxy("ALTextToSpeech",NAOIP,PORT)

# # Connect to the module ALRobotPostureProxy
# postObj = ALProxy("ALRobotPosture",NAOIP,PORT)

# # Connect to the module ALVideoDeviceProxy
# vidObj = ALProxy("ALVideoDevice",NAOIP,PORT)

# # Connect to the module ALMemoryProxy
# memProxy = ALProxy("ALMemory",NAOIP,PORT)

# # Connect to the module DCM
# dcm = ALProxy("DCM",NAOIP,PORT)

# # Connect to the module ALLeds
# ledObj = ALProxy("ALLeds", NAOIP, PORT)



#############



#############

#memProxy.subscribeToEvent("RightBumperPressed", "Bumper","onRightBumperPressed");


#memProxy.insertData("Device/SubDeviceList/LShoulderPitch/Position/Actuator/IMax",350); #70
#memProxy.insertData("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Kd",100);
#memProxy.insertData("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Kp",100);


 
######################################
# ALVideoDeviceProxy
# subscriberID = 'matlab'
# fps = 30
# Resolution = 2
# ColorSpace = 13
# CameraIndex = 0
# vidObj.subscribeCamera(subscriberID,CameraIndex ,Resolution ,ColorSpace, fps)
# vidObj.setParam(18, 0)
# vidObj.setFrameRate(subscriberID,10)

# Image container is an array as follow: (indexing is in cpp)
    #[0]: width.
    #[1]: height.
    #[2]: number of layers.
    #[3]: ColorSpace.
    #[4]: time stamp (seconds).
    #[5]: time stamp (micro-seconds).
    #[6]: array of size height * width * nblayers containing image data.
    #[7]: camera ID (kTop=0, kBottom=1).
    #[8]: left angle (radian).
    #[9]: topAngle (radian).
    #[10]: rightAngle (radian).
    #[11]: bottomAngle (radian).    
######################################
   
#postObj.goToPosture('Crouch',0.9) # Crouch Stand
#time.sleep(1)

#TextObj.say('Please give me the ball'); 
#pause(4);
#movObj.setAngles('LHand', 0, 0.5);
#pause(2);

# Control "RElbowYaw" joint 
fractionMaxSpeed = 1.0

# Disable Fall Manager 
# TextObj.say('Attention, Fall Manager is Disabled.')
# movObj.setFallManagerEnabled(False) # True False
# time.sleep(1)

# movObj.setStiffnesses('Body',0.5);
# movObj.setAngles('Body', zeros(number_cpg,1), fractionMaxSpeed/10);
# pause(5);

# Move the arm into home-throwing-position 
#teta=[0.0*math.pi/180.0, 30.0*math.pi/180.0, -90.0*math.pi/180.0, -85.0*math.pi/180.0, 90.0*math.pi/180.0, 1] 

#teta=[0.0*math.pi/180.0, 30.0*math.pi/180.0, -90.0*math.pi/180.0, -85.0*math.pi/180.0, 90.0*math.pi/180.0, 1] 

#teta=[20 0 -90 -85 90 0];

#movObj.setAngles('LArm', teta , 0.3)

#pause(2);

# http://doc.aldebaran.com/2-1/family/robots/postures_robot.html#robot-postures
#postObj.goToPosture("StandInit",0.8)
#time.sleep(2)

# ALMemory 
ReadlistData = [
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
              "Device/SubDeviceList/RHand/Touch/Right/Sensor/Value", #8 
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
           ]


#listValRead = memProxy.getListData( ReadlistData )
#print "listVal:" ,listValRead 
#print "listVal[0]:" ,listValRead[0] 
#print "listVal[1]:" ,listValRead[1] 

###################################### 

myT = fSetTiming()
#fPrintTiming(myT)

myCont = fnewMLMPcpg(number_cpg)
#fPrintMLMPcpg(myCont[0:number_cpg])
print('myCont', myCont)
myCont = fSetCPGNet(myCont,'MyNao.txt','MyNaoPsitiveAngle_E_or_F.txt')
#myCont2[25].RG.E.Es = 99
#fPrintMLMPcpg(myCont)

PatternOsc0 = RG_Patterns(10,10,1,0.1)
PatternOsc1 = RG_Patterns(17.5,17.5,1,0.05)
PatternOsc2 = RG_Patterns(10,10,1,0.1) 

PatternOsc3 = RG_Patterns(2,10,1,0.1) # This is a smooth patern 
PatternOsc = PatternOsc1


PatternOsc4 = RG_Patterns(1.5,10,1,0.1) 

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
PatternPL1 = RG_Patterns(5, 0.1, 1, 0.1)
PatternPL2 = RG_Patterns(5, 0.1, 1, 0.4)
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
LSP_PFPattern = PF_Patterns(0.0,0)

LHR_PFPattern = PF_Patterns(0.022,0)
RHR_PFPattern = LHR_PFPattern

LAR_PFPattern = PF_Patterns(0.022,0)
RAR_PFPattern = LAR_PFPattern


LHP_PFPattern = PF_Patterns(0.02,0)
RHP_PFPattern = LHP_PFPattern

LKP_PFPattern = PF_Patterns(0.015,0) # or 0.02
RKP_PFPattern = LKP_PFPattern

LAP_PFPattern = PF_Patterns(0.00,0)
RAP_PFPattern = LAP_PFPattern


#LSP_PFPattern.fPrint()

"""
myCont[L_SHOULDER_PITCH].fSetPatternPF(LSP_PFPattern)
myCont[L_SHOULDER_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[L_SHOULDER_ROLL].fSetPatternPF(LSP_PFPattern)
myCont[L_SHOULDER_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_SHOULDER_PITCH].fSetPatternPF(LSP_PFPattern)
myCont[R_SHOULDER_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_SHOULDER_ROLL].fSetPatternPF(LSP_PFPattern)
myCont[R_SHOULDER_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU
"""

# for roll legs motors 
# HIP
myCont[L_HIP_ROLL].fSetPatternPF(LHR_PFPattern)
myCont[L_HIP_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_HIP_ROLL].fSetPatternPF(RHR_PFPattern)
myCont[R_HIP_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

# ANKLE
myCont[L_ANKLE_ROLL].fSetPatternPF(LAR_PFPattern)
myCont[L_ANKLE_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_ANKLE_ROLL].fSetPatternPF(RAR_PFPattern)
myCont[R_ANKLE_ROLL].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

# for pitch legs motors 
# Right
myCont[R_HIP_PITCH].fSetPatternPF(RHP_PFPattern)
myCont[R_HIP_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_KNEE_PITCH].fSetPatternPF(RKP_PFPattern)
myCont[R_KNEE_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[R_ANKLE_PITCH].fSetPatternPF(RAP_PFPattern)
myCont[R_ANKLE_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

# Left
myCont[L_HIP_PITCH].fSetPatternPF(LHP_PFPattern)
myCont[L_HIP_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[L_KNEE_PITCH].fSetPatternPF(LKP_PFPattern)
myCont[L_KNEE_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

myCont[L_ANKLE_PITCH].fSetPatternPF(LAP_PFPattern)
myCont[L_ANKLE_PITCH].fSetPatternRG(PatternOsc) # PatternPL1 PatternOsc3 PatternAosc PatternQU

#myCont[L_SHOULDER_PITCH].fPrint()

"""
HEAD_YAW,HEAD_PITCH,L_SHOULDER_PITCH,L_SHOULDER_ROLL,L_ELBOW_YAW,L_ELBOW_ROLL,
L_WRIST_YAW,L_HAND,L_HIP_YAW_PITCH,L_HIP_ROLL,L_HIP_PITCH,L_KNEE_PITCH,L_ANKLE_PITCH,
L_ANKLE_ROLL,R_HIP_YAW_PITCH,R_HIP_ROLL,R_HIP_PITCH,R_KNEE_PITCH,R_ANKLE_PITCH,
R_ANKLE_ROLL,R_SHOULDER_PITCH,R_SHOULDER_ROLL,R_ELBOW_YAW,R_ELBOW_ROLL,
R_WRIST_YAW,R_HAND
"""
CurPos = NaoConnect.NaoGetAngles()

for i in range(0, len(myCont)):
    myCont[i].fUpdateInitPos(CurPos[i])

myCont[L_HIP_ROLL].fUpdateInitPos(2*math.pi/180.0)
myCont[R_HIP_ROLL].fUpdateInitPos(-2*math.pi/180.0)
myCont[L_ANKLE_ROLL].fUpdateInitPos(2*math.pi/180.0)
myCont[R_ANKLE_ROLL].fUpdateInitPos(-2*math.pi/180.0)

"""
 9. LHipYawPitch* 	Left hip joint twist (Y-Z 45deg)          -65.62 to 42.44 	-1.145303 to 0.740810
10. LHipRoll      Left hip joint right and left (X)           -21.74 to 45.29 	-0.379472 to 0.790477
11. LHipPitch 	Left hip joint front and back (Y)           -88.00 to 27.73 	-1.535889 to 0.484090
12. LKneePitch 	Left knee joint (Y)                         -5.29 to 121.04 	-0.092346 to 2.112528
13. LAnklePitch 	Left ankle joint front and back (Y)         -68.15 to 52.86 	-1.189516 to 0.922747
14. LAnkleRoll 	Left ankle joint right and left (X)         -22.79 to 44.06 	-0.397880 to 0.769001
 """

myCont[L_HIP_PITCH].fUpdateInitPos(-0.4) # -0.4
myCont[R_HIP_PITCH].fUpdateInitPos(myCont[L_HIP_PITCH].joint.init_motor_pos)

myCont[L_KNEE_PITCH].fUpdateInitPos(0.7) # 0.7
myCont[R_KNEE_PITCH].fUpdateInitPos(myCont[L_KNEE_PITCH].joint.init_motor_pos)

myCont[L_ANKLE_PITCH].fUpdateInitPos(-0.35) # -0.35
myCont[R_ANKLE_PITCH].fUpdateInitPos(myCont[L_ANKLE_PITCH].joint.init_motor_pos)



##print len(CurPos)  

#for i in range(0, len(myCont)):
#    myCont[i].fPrint()
 
myCont[L_SHOULDER_PITCH].W_E_SN2MN=-0.0
myCont[L_SHOULDER_PITCH].W_F_SN2MN=-0.0

myCont[L_SHOULDER_ROLL].W_E_SN2MN=-0.0
myCont[L_SHOULDER_ROLL].W_F_SN2MN=-0.0
    
#############################
for i in range(0, len(myCont)):
    myCont[i].fUpdateLocomotionNetwork(myT,CurPos[i])
#############################
time1 = time.time()

readOscFreq = 0
readOscFreq2 = 0
readOscFreqOverTime = []

readLFSRFreq = 0
readLFSRFreq2 = 0
readLFSRFreqOverTime = [] 

readRFSRFreq = 0
readRFSRFreq2 = 0
readRFSRFreqOverTime = [] 


RFsr = 2 # This value is an extinmation from standin postion 
LFsr = 2 # This value is an extinmation from standin postion 

L_HIP_ROLL_RG_E_out = 0

MAT_Iinj=[];
for I in range(0,int(myT.N_Loop/2)):
    t= I*myT.T;

    # #################
    # ExtInjCurr
    # #################
    if t>= myT.T1 and t < myT.T2:
        ExtInjCurr = 1
    #elif t >= myT.T3 and t < myT.T4:
    #    ExtInjCurr = 4.5
    #elif t >= myT.T5 and t < myT.T6:    
    #    ExtInjCurr = -4.5
    else: 
        ExtInjCurr = 0
        
    if t>= myT.T3 and t < myT.T4:
        ExtInjCurr2 = 1
    #elif t >= myT.T3 and t < myT.T4:
    #    ExtInjCurr = 4.5
    #elif t >= myT.T5 and t < myT.T6:    
    #    ExtInjCurr = -4.5
    else: 
        ExtInjCurr2 = 0

    MAT_Iinj.append(ExtInjCurr) 

    #L_SHOULDER_PITCH , L_SHOULDER_ROLL,
    for ii in [ L_HIP_ROLL, L_ANKLE_ROLL]:
        myCont[ii].RG.F.InjCurrent_value = +1*ExtInjCurr* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value = -1*ExtInjCurr* myCont[ii].RG.E.InjCurrent_MultiplicationFactor
    #R_SHOULDER_PITCH, R_SHOULDER_ROLL,
    for ii in [ R_HIP_ROLL, R_ANKLE_ROLL]:
        myCont[ii].RG.F.InjCurrent_value = -1*ExtInjCurr* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value = +1*ExtInjCurr* myCont[ii].RG.E.InjCurrent_MultiplicationFactor


    for ii in [R_HIP_PITCH, R_KNEE_PITCH, L_ANKLE_PITCH]:
        myCont[ii].RG.F.InjCurrent_value = +1*ExtInjCurr2* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value = -1*ExtInjCurr2* myCont[ii].RG.E.InjCurrent_MultiplicationFactor

    for ii in [L_HIP_PITCH, L_KNEE_PITCH, R_ANKLE_PITCH]:
        myCont[ii].RG.F.InjCurrent_value = -1*ExtInjCurr2* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value = +1*ExtInjCurr2* myCont[ii].RG.E.InjCurrent_MultiplicationFactor

    """
    for i in range(0, len(myCont)):
        myCont[i].fUpdateLocomotionNetwork(myT,CurPos[i])
    """
    
    OLD_L_HIP_ROLL_RG_E_out = myCont[L_HIP_ROLL].RG.E.out

    for i in [L_HIP_ROLL, L_ANKLE_ROLL,R_HIP_ROLL, R_ANKLE_ROLL,R_HIP_PITCH, R_KNEE_PITCH, L_ANKLE_PITCH,L_HIP_PITCH, L_KNEE_PITCH, R_ANKLE_PITCH]:
        myCont[i].fUpdateLocomotionNetwork(myT,CurPos[i])

        
    for i in range(0, len(myCont)):
        MotorCommand[i]=myCont[i].joint.joint_motor_signal
    
    NaoConnect.NaoSetAngles(MotorCommand)
    ##movObj.angleInterpolation('Body',MotorCommand , 0.03, True)
    

    # time.sleep(myT.T)
    
     # Read motor angel from robot joints 
    CurPos = NaoConnect.NaoGetAngles()
    
    All_Command.append(MotorCommand[:]) 
    All_Sensor.append(CurPos)
    
    #data2print1.append(myCont[L_HIP_ROLL].SN.E.o)
    #data2print2.append(myCont[L_HIP_ROLL].SN.F.o)
    
    ######################################
    
    # listValRead = memProxy.getListData( ReadlistData )

    # # "RearTactileON" stop 
    # if (listValRead[1]):
    #     print 'Walking has been stoped by toutching the rear tactile head sensor.\n'
    #     break
    
    """    
    # "MiddleTactileON" CLOSE THE HAND TO PICK THE BALL 
    MiddleTactileON = 0 
    MiddleTactileON = memProxy.getData('Device/SubDeviceList/Head/Touch/Rear/Sensor/Value')
    if (MiddleTactileON):
        print 'Walking has been stoped by toutching the rear tactile head sensor.\n'
        break
    """
    ###################################### 
    # Msurement of hand toutch sensors (Tactile Hands)
    # http://doc.aldebaran.com/2-1/family/robots/contact-sensors_robot.html#robot-contact-hand 
    """
    LHandBackSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Back/Sensor/Value')
    LHandLeftSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Left/Sensor/Value')
    LHandRightSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Right/Sensor/Value')

    RHandBackSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Back/Sensor/Value')
    RHandLeftSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Left/Sensor/Value')
    RHandRightSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Right/Sensor/Value')
    """

    # LHandBackSensor = listValRead[3]
    # LHandLeftSensor = listValRead[4]
    # LHandRightSensor = listValRead[5]

    # RHandBackSensor = listValRead[6]
    # RHandLeftSensor = listValRead[7]
    # RHandRightSensor = listValRead[8]
    
    #print 'LHandSensor ',LHandBackSensor,LHandLeftSensor,LHandRightSensor
    #print 'RHandSensor ',RHandBackSensor,RHandLeftSensor,RHandRightSensor

    #print 'LHandBack:', LHandBackSensor,' LHandLeft:',LHandLeftSensor,' LHandRight:',LHandRightSensor,' RHandBack:',RHandBackSensor,' RHandLeft:',RHandLeftSensor,' RHandRight:',RHandRightSensor
    """ 
    if (LHandBackSensor==1 or LHandLeftSensor==1  or LHandRightSensor==1 ):
        movObj.setStiffnesses('LArm',0.0)
    else:
        movObj.setStiffnesses('LArm',0.1)
    """    
    # movObj.setStiffnesses('LArm', 0.5 * int( not ((LHandBackSensor == 1) or (LHandLeftSensor == 1)  or (LHandRightSensor == 1))))
    
    """    
    if (RHandBackSensor==1  or RHandLeftSensor==1  or RHandRightSensor==1 ):
        movObj.setStiffnesses('RArm',0.0)
    else:
        movObj.setStiffnesses('RArm',0.1)
    """
    # movObj.setStiffnesses('RArm', 0.5 * int( not ((RHandBackSensor == 1) or (RHandLeftSensor == 1)  or (RHandRightSensor == 1))))
    
    ######################################
    """
    if (LHandBackSensor): 
        ledObj.on("LeftEarsMidle") 
    else:
        ledObj.off("LeftEarsMidle")

    if (LHandLeftSensor): 
        ledObj.on("LeftEarsFront") 
    else:
        ledObj.off("LeftEarsFront")
    
    if (LHandRightSensor): 
        ledObj.on("LeftEarsBack") 
    else:
        ledObj.off("LeftEarsBack")


    if (RHandBackSensor): 
        ledObj.on("RightEarsMidle") 
    else:
        ledObj.off("RightEarsMidle")

    if (RHandLeftSensor): 
        ledObj.on("RightEarsBack") 
    else:
        ledObj.off("RightEarsBack")
    
    if (RHandRightSensor): 
        ledObj.on("RightEarsFront") 
    else:
        ledObj.off("RightEarsFront")
    """

    #ledObj.on("LeftEarsFront")LeftEarsMidle LeftEarsBack
    ######################################
    
    # Get The Left Foot Force Sensor Values
    """   
    LFsrFL = memProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    """
    # LFsrFL = listValRead[9]
    # LFsrFR = listValRead[10]
    # LFsrBL = listValRead[11]
    # LFsrBR = listValRead[12]
    # old_LFsr = LFsr
    # LFsr = LFsrFL+ LFsrFR+ LFsrBL+ LFsrBR

    #print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    #print( "Left FSR [Kg] : %.2f " %  (LFsrFL+ LFsrFR+ LFsrBL+ LFsrBR) )
    
    # Get The Right Foot Force Sensor Values
    """
    RFsrFL = memProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    """

    # RFsrFL = listValRead[13]
    # RFsrFR = listValRead[14]
    # RFsrBL = listValRead[15]
    # RFsrBR = listValRead[16]
    # old_RFsr = RFsr
    # RFsr = RFsrFL+ RFsrFR+ RFsrBL+ RFsrBR


    #print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    #print( "Right FSR [Kg] : %.2f " %  (RFsrFL+ RFsrFR+ RFsrBL+ RFsrBR) )
    
    #print( "Left FSR [Kg] : %.2f " %  (LFsrFL+ LFsrFR+ LFsrBL+ LFsrBR),  "Right FSR [Kg] : %.2f " %  (RFsrFL+ RFsrFR+ RFsrBL+ RFsrBR) )
    
    
    ############################
    ############################
    # Optimization from user... 
    """
    if (myCont[L_HIP_ROLL].RG.E.out>0.0):
        readOscFreq = readOscFreq+1    
        readOscFreqOverTime.append(readOscFreq)
        readOscFreq2 = 0
    
    if (myCont[L_HIP_ROLL].RG.E.out<-0.0):
        readOscFreq2 = readOscFreq2-1 
        readOscFreqOverTime.append(readOscFreq2)
        readOscFreq = 0
    """
    # if ( (OLD_L_HIP_ROLL_RG_E_out<0)and (myCont[L_HIP_ROLL].RG.E.out>0)):
    #     print 'readOscFreq = ',readOscFreq
    #     readOscFreq = 0
    # readOscFreqOverTime.append(readOscFreq)
    # readOscFreq = readOscFreq + 1
        
    # ############################
    # if ((old_LFsr<0.5)and(LFsr>0.5)):        
    #     if(readLFSRFreq>9):
    #         print 'readLFSRFreq =                ',readLFSRFreq
    #         readLFSRFreqOverTime.append(readLFSRFreq)
    #         readLFSRFreq = 0
    # readLFSRFreq = readLFSRFreq + 1
        
    # ############################
    # if ((old_RFsr<0.5)and(RFsr>0.5)):        
    #     if(readRFSRFreq>9):
    #         print 'readRFSRFreq =                       ',readRFSRFreq
    #         readRFSRFreqOverTime.append(readRFSRFreq)
    #         readRFSRFreq = 0
    # readRFSRFreq = readRFSRFreq + 1
        
    """
    if (LFsr > 2):
        readFSRFreq = readFSRFreq+1
        readFSRFreqOverTime.append(readFSRFreq)
        readFSRFreq2 = 0

    if (LFsr < 2):
        readFSRFreq2 = readFSRFreq2-1
        readFSRFreqOverTime.append(readFSRFreq2)
        readFSRFreq = 0
    """
    #readFSRFreqOverTime.append(LFsr)
    
    """
    if (LFsr > RFsr):
        readFSRFreq = readFSRFreq+1    
        readFSRFreqOverTime.append(readFSRFreq)
        readFSRFreq2 = 0
        
    if (LFsr < RFsr):
        readFSRFreq2 = readFSRFreq2-1    
        readFSRFreqOverTime.append(readFSRFreq2)
        readFSRFreq = 0
    """    
    ############################
    time2 = time.time()
    while (time2 - time1)<0.04 :
        time2 = time.time()
    #print 'time diff: ', time2 - time1
    #time_diff  = time2 - time1
    time1 = time2 
    #print 'readOscFreqOverTime=',readOscFreqOverTime

    #print 'readFSRFreqOverTime=',readFSRFreqOverTime
    
    #print ' '

#print 'readOscFreqOverTime=',readOscFreqOverTime

#print 'readLFSRFreqOverTime=',readLFSRFreqOverTime

#print 'time diff: ', time_diff
##############################

# postObj.goToPosture("Crouch",0.8)
# time.sleep(2)
# movObj.setStiffnesses('Body',0.0)


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


#plt.plot( range(len(data2print1)), data2print1,'b', range(len(data2print2)),data2print2,'r')
#plt.grid(True)
#plt.show()


#fPlotJointCommandSensor(All_Command,All_Sensor,L_SHOULDER_ROLL,'L_SHOULDER_ROLL')
#fPlotJointCommandSensor(All_Command,All_Sensor,L_SHOULDER_PITCH,'L_SHOULDER_PITCH')


#print MAT_Iinj
""""    
        if (t >= myTime.T1 && t < myTime.T2) 
        ExtInjCurr = 1;
  %  elseif (t >= myTime.T3 && t < myTime.T4)
  %      ExtInjCurr = 4.5;
  %  elseif (t >= myTime.T5 && t < myTime.T6)
  %      ExtInjCurr = -4.5;        
    else 
        ExtInjCurr = 0;
    end
    MAT_Iinj=[MAT_Iinj, ExtInjCurr];
    if (t > myTime.T2) 
    movObj.setStiffnesses('LArm',0.8);
    end
    
"""

#memProxy.unsubscribeToEvent("onRightBumperPressed", "Bumper");



