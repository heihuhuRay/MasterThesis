# -*- coding: utf-8 -*-
"""
This module contains the inisialization code to connect to the real robot. It also has functions to read and set nao robot joints and also read robot sonsors. 
It is a middlewear between the real robot and the user code. 

Created on Mon May 29 01:17:53 2017

@todo: all

@author: nasjo
"""
import time


global RealNaoRobot

global postObj

NaoQi = []
RealNaoRobot = []

NAOIP = []
PORT = []


fractionMaxSpeed = 1.0
listValRead = []

######################################
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

######################################
           
#### NAOQI 
try: 
    import naoqi
    from naoqi import ALProxy
    from naoqi import ALModule
    NaoQi.append(1)
    print ('Naoqi is imported.')            
except:
    print ('-----------------------------')
    print ('Error in importing naoqi!')
    print ('Make sure naoqi is installed')
    print ('-----------------------------')
    print ('')


if NaoQi:
    
    try: 
        NAOIP = "nao.local"
        PORT= 9559
        movObj = ALProxy("ALMotion",NAOIP,PORT)
        postObj = ALProxy("ALRobotPosture",NAOIP,PORT)
        memProxy = ALProxy("ALMemory",NAOIP,PORT)
        
        movObj.setStiffnesses('Body',0.99)
        listValRead = memProxy.getListData( ReadlistData )

        RealNaoRobot.append(1)
        print ('Connect to the real robot.')        
    except: 
        print ('Cannot connect to the real robot!')



######################################################################

def NaoSetAnglesRobot(angles):
    
    # print "NAOIP", NAOIP
    movObj.setAngles('Body',angles[0:26], fractionMaxSpeed)
    return
######################################################################
    
def NaoGetAnglesRobot():
    
    movObj.getAngles('Body',True)
    return movObj.getAngles('Body',True) 

######################################################################
def NaoGetSensorsRobot():
    listValRead = memProxy.getListData( ReadlistData )
    FsrLeft = listValRead[9:13]
    FsrRight = listValRead[13:17]
    robPos = [0,0,0]
    robOrient = [ listValRead[17] , listValRead[18], 0]
    HeadTouch = listValRead[0:3]   
    HandTouchLeft = listValRead[3:6]   
    HandTouchRight = listValRead[6:9]   

    return (FsrLeft,FsrRight,robPos,robOrient,HeadTouch,HandTouchLeft,HandTouchRight)

######################################################################
def NaoStopRobot():
    #StandInit
    #StandZero
    #Crouch    
    postObj.goToPosture("Crouch",0.8)
    time.sleep(2)
    movObj.setStiffnesses('Body',0.0)

    return

   