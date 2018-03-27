# -*- coding: utf-8 -*-
"""
This module contains the inisialization code to connect to Vrep simulator. It also has functions to read and set nao robot joints and also read sonsors in Vrep simulation. 
It is a middlewear between the simulator and the user code. 

Created on Mon May 29 00:29:11 2017

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

import numpy 
import time 

NaoJointsName=['HeadYaw','HeadPitch','LShoulderPitch','LShoulderRoll','LElbowYaw','LElbowRoll','LWristYaw','LHand','LHipYawPitch',
                'LHipRoll','LHipPitch','LKneePitch','LAnklePitch','LAnkleRoll','RHipYawPitch','RHipRoll','RHipPitch','RKneePitch',
                'RAnklePitch','RAnkleRoll','RShoulderPitch','RShoulderRoll','RElbowYaw','RElbowRoll','RWristYaw','RHand']

NaoLFsrName=['NAO_LFsrFL','NAO_LFsrFR','NAO_LFsrRL','NAO_LFsrRR']
NaoRFsrName=['NAO_RFsrFL','NAO_RFsrFR','NAO_RFsrRL','NAO_RFsrRR']
NaoOrientationName=['NAO']
NaoPositionsName=['NAO']
NaoVisionName=['NAO_vision1','NAO_vision2']

NaojointHandles=[0]*len(NaoJointsName)
NaoFsrLHandles=[0]*len(NaoLFsrName)
NaoFsrRHandles=[0]*len(NaoRFsrName)
NaoOrientationHandles=[0]*len(NaoOrientationName)
NaoPositionHandles=[0]*len(NaoPositionsName)
NaoVisionHandles=[0]*len(NaoVisionName)


global NaoVrep
NaoVrep = []
global clientID

clientID=-1


######################################################################

#def NaoSetAnglesVrep(angles=numpy.zeros(len(NaoJointsName))):
def NaoSetAnglesVrep(angles):
    #print "NaojointHandles: ",NaojointHandles
    for i in range(len(NaoJointsName)):
        #[Handle] = NaojointHandles[i]
        #print NaojointHandles[i]
        vrep.simxSetJointTargetPosition(clientID, NaojointHandles[i], angles[i], vrep.simx_opmode_oneshot)
    #print angles
    return
    

######################################################################
def NaoGetAnglesVrep():
            
    ang = [0.0]*len(NaoJointsName)
    i=0
    for i in range(len(NaoJointsName)):
        if NaojointHandles[i]!=0:
            #print NaojointHandles[i]
        
            """
            ErrorNao = 1
            while(ErrorNao):
                ErrorNao, ang[i]   = vrep.simxGetJointPosition(clientID, NaojointHandles[i], vrep.simx_opmode_oneshot)
                if ErrorNao:
                    print "ErrorNao is on in NaoGetAnglesVrep.."     
            """
            ErrorNao, ang[i]   = vrep.simxGetJointPosition(clientID, NaojointHandles[i], vrep.simx_opmode_oneshot)

    return ang 
    
    
def NaoStopVrep(): 
    # no thing to do so far !
    return

######################################################################
def NaoGetSensorsVrep():

    sens = [0.0]* len(NaoLFsrName)
    for i in range(len(NaoLFsrName)):
        if NaoFsrLHandles[i]!=0:
            (d1,d2,forces,torques) = vrep.simxReadForceSensor(clientID, NaoFsrLHandles[i], vrep.simx_opmode_oneshot)
            sens[i] = forces[2]
    FsrLeft = sens
    
    sens = [0.0]* len(NaoRFsrName)
    for i in range(len(NaoRFsrName)):
        if NaoFsrRHandles[i]!=0:
            (d1,d2,forces,torques) = vrep.simxReadForceSensor(clientID, NaoFsrRHandles[i], vrep.simx_opmode_oneshot)
            sens[i] = forces[2]
    FsrRight = sens


    sens = [0.0]* len(NaoPositionsName)
    for  i in range(len(NaoPositionsName)):
        (d,sens[i]) = vrep.simxGetObjectPosition(clientID, NaoPositionHandles[i], -1,vrep.simx_opmode_oneshot)
    [robPos] = sens

    sens = [0.0]* len(NaoOrientationName)
    for  i in range(len(NaoOrientationName)):
        (d,sens[i]) = vrep.simxGetObjectOrientation(clientID, NaoOrientationHandles[i], -1,vrep.simx_opmode_oneshot)
    [robOrient] = sens

    
    HeadTouch = [0,0,0] # No HeadTouch 
    HandTouchLeft = [0,0,0] # No Hand Touch Left
    HandTouchRight = [0,0,0] # No Hand Touch Right  

    return (FsrLeft,FsrRight,robPos,robOrient,HeadTouch,HandTouchLeft,HandTouchRight)
    




#### V-REP
try:
    
    import vrep
    print ('vrep is imported.')            
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
    if clientID!=-1:
        print ('Connection to V-rep established')
        

        for i in range(len(NaoJointsName)):
            ErrorNao, NaojointHandles[i]   = vrep.simxGetObjectHandle(clientID,NaoJointsName[i],vrep.simx_opmode_oneshot_wait)
            if ErrorNao: 
                print "Cannot creat a handle to --> ",NaoJointsName[i]
        
        
        for i in range(len(NaoLFsrName)):
            ErrorNao, NaoFsrLHandles[i]   = vrep.simxGetObjectHandle(clientID,NaoLFsrName[i],vrep.simx_opmode_oneshot_wait)
            if ErrorNao: 
                print "Cannot creat a handle to --> ",NaoLFsrName[i]

        for i in range(len(NaoRFsrName)):
            ErrorNao, NaoFsrRHandles[i]   = vrep.simxGetObjectHandle(clientID,NaoRFsrName[i],vrep.simx_opmode_oneshot_wait)
            if ErrorNao: 
                print "Cannot creat a handle to --> ",NaoRFsrName[i]

        for i in range(len(NaoOrientationName)):
            ErrorNao, NaoOrientationHandles[i]   = vrep.simxGetObjectHandle(clientID,NaoOrientationName[i],vrep.simx_opmode_oneshot_wait)
            if ErrorNao: 
                print "Cannot creat a handle to --> ",NaoOrientationName[i]

        for i in range(len(NaoPositionsName)):
            ErrorNao, NaoPositionHandles[i]   = vrep.simxGetObjectHandle(clientID,NaoPositionsName[i],vrep.simx_opmode_oneshot_wait)
            if ErrorNao: 
                print "Cannot creat a handle to --> ",NaoPositionsName[i]


        for i in range(len(NaoVisionName)):
            ErrorNao, NaoVisionHandles[i]   = vrep.simxGetObjectHandle(clientID,NaoVisionName[i],vrep.simx_opmode_oneshot_wait)
            if ErrorNao: 
                print "Cannot creat a handle to --> ",NaoVisionName[i]


        NaoVrep.append(1)

    else: 
        print ('Cannot connect to V-rep!')

except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')


# Run NaoGetAnglesVrep() two times at the start ... initialization 
# 
NaoGetAnglesVrep()
time.sleep(1)
NaoGetAnglesVrep()
time.sleep(1)







