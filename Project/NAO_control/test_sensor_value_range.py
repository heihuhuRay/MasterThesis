#!/usr/bin/env python
# -*- coding: utf-8 -*-
# __date__ = 20180131
# modified by: Ray

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
# init max and min value for each sensor
min_R = [1023, 1023, 1023, 1023, 1023, 1023, 1023]
max_R = [0, 0, 0, 0, 0, 0, 0]

min_L = [1023, 1023, 1023, 1023, 1023, 1023, 1023]
max_L = [0, 0, 0, 0, 0, 0, 0]

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
    # Example Data
    #    [[778, 868, 867, 848, 901, 831, 554], [411, 769, 863, 621, 772, 678, 699]]
    #    [[R_1, R_2, R_3, R_4, R_5, R_6, R_7], [L_1, L_2, L_3, L_4, L_5, L_6, L_7]]
    print('Right_Wrist_Sensor,                  Left_Wrist_Sensors')
    print("[[R_1, R_2, R_3, R_4, R_5, R_6, R_7], [L_1, L_2, L_3, L_4, L_5, L_6, L_7]]")
    print(data)


    ########### calc range for each sensor ###########################
    # update max_R
    if data[0][0] > max_R[0]:
        max_R[0] = data[0][0]
    if data[0][1] > max_R[1]:
        max_R[1] = data[0][1]
    if data[0][2] > max_R[2]:
        max_R[2] = data[0][2]
    if data[0][3] > max_R[3]:
        max_R[3] = data[0][3]
    if data[0][4] > max_R[4]:
        max_R[4] = data[0][4]
    if data[0][5] > max_R[5]:
        max_R[5] = data[0][5]
    if data[0][6] > max_R[6]:
        max_R[6] = data[0][6]
    # update min_R
    if data[0][0] < min_R[0]:
        min_R[0] = data[0][0]
    if data[0][1] < min_R[1]:
        min_R[1] = data[0][1]
    if data[0][2] < min_R[2]:
        min_R[2] = data[0][2]
    if data[0][3] < min_R[3]:
        min_R[3] = data[0][3]
    if data[0][4] < min_R[4]:
        min_R[4] = data[0][4]
    if data[0][5] < min_R[5]:
        min_R[5] = data[0][5]
    if data[0][6] < min_R[6]:
        min_R[6] = data[0][6]

    # update max_L
    if data[1][0] > max_L[0]:
        max_L[0] = data[1][0]
    if data[1][1] > max_L[1]:
        max_L[1] = data[1][1]
    if data[1][2] > max_L[2]:
        max_L[2] = data[1][2]
    if data[1][3] > max_L[3]:
        max_L[3] = data[1][3]
    if data[1][4] > max_L[4]:
        max_L[4] = data[1][4]
    if data[1][5] > max_L[5]:
        max_L[5] = data[1][5]
    if data[1][6] > max_L[6]:
        max_L[6] = data[1][6]
    # update min_L
    if data[1][0] < min_L[0]:
        min_L[0] = data[1][0]
    if data[1][1] < min_L[1]:
        min_L[1] = data[1][1]
    if data[1][2] < min_L[2]:
        min_L[2] = data[1][2]
    if data[1][3] < min_L[3]:
        min_L[3] = data[1][3]
    if data[1][4] < min_L[4]:
        min_L[4] = data[1][4]
    if data[1][5] < min_L[5]:
        min_L[5] = data[1][5]
    if data[1][6] < min_L[6]:
        min_L[6] = data[1][6]

    time.sleep(0.1)

print('min_R', min_R)
print('max_R', max_R)
print('min_L', min_L)
print('max_L', max_L)

# normalization
range_R = [ max_R[0] - min_R[0],
            max_R[1] - min_R[1],
            max_R[2] - min_R[2],
            max_R[3] - min_R[3],
            max_R[4] - min_R[4],
            max_R[5] - min_R[5],
            max_R[6] - min_R[6],
            ]

range_L = [ max_L[0] - min_L[0],
            max_L[1] - min_L[1],
            max_L[2] - min_L[2],
            max_L[3] - min_L[3],
            max_L[4] - min_L[4],
            max_L[5] - min_L[5],
            max_L[6] - min_L[6],
            ]

print('range_R:', range_R)
print('range_L:', range_L)

R_nor_data = [  (data[0][0] - min_R[0])/range_R[0],
                (data[0][1] - min_R[1])/range_R[1],
                (data[0][2] - min_R[2])/range_R[2],
                (data[0][3] - min_R[3])/range_R[3],
                (data[0][4] - min_R[4])/range_R[4],
                (data[0][5] - min_R[5])/range_R[5],
                (data[0][6] - min_R[6])/range_R[6],
            ]

L_nor_data = [  (data[1][0] - min_L[0])/range_L[0],
                (data[1][1] - min_L[1])/range_L[1],
                (data[1][2] - min_L[2])/range_L[2],
                (data[1][3] - min_L[3])/range_L[3],
                (data[1][4] - min_L[4])/range_L[4],
                (data[1][5] - min_L[5])/range_L[5],
                (data[1][6] - min_L[6])/range_L[6],
            ]

#print("[[R_1, R_2, R_3, R_4, R_5, R_6, R_7], [L_1, L_2, L_3, L_4, L_5, L_6, L_7]]")
#print("[", R_nor_data, L_nor_data, "]")

movObj.setStiffnesses('HeadYaw',0.0)
