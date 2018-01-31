import socket
import time
import threading
import ast
import naoqi
from naoqi import ALProxy
import math 
import numpy as np
import os
import sys
from naoqi import ALProxy

######################################
NAOIP = "0.0.0.0"
PORT= 9559

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
           ]
######################################
# Connect to the module ALMemoryProxy
memProxy = ALProxy("ALMemory", NAOIP, PORT)
listValRead = memProxy.getListData(ReadlistData)
print('head sensor_touch:', listValRead[1])

time.sleep(2) 

while True:
    # 摸摸头
    if (listValRead[1]):
        print('Testing has been stoped by toutching the rear tactile head sensor.\n')
        break

    FrontTactileON   = memProxy.getData('Device/SubDeviceList/Head/Touch/Front/Sensor/Value')
    if FrontTactileON == 1:
        break 

    data = memProxy.getData("WristForceSensor")
    print('LeftSens,           RightSens')
    print(data)
    
    #print data2[4]

    #print data2[4]
    #print data2[0]
    #print "--"
    #print data2[1]
    
    time.sleep(0.1)

movObj.setStiffnesses('HeadYaw',0.0)
