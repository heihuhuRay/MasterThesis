#!/usr/bin/env python
# -*- coding: utf-8 -*-
# __date__ = 20180131
# modified by: Ray


'''
Run on pi
'''
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
import matplotlib.pyplot as plt
import matplotlib.animation as animation

######################################
NAOIP = "0.0.0.0"
PORT= 9559
# experiment data based on result from test_sendor_value_range.py
min_R = [491, 670, 708, 685, 769, 714, 416]
max_R = [766, 889, 878, 873, 855, 818, 548]
min_L = [287, 530, 645, 431, 377, 473, 554]
max_L = [524, 784, 945, 879, 812, 769, 935]
range_R = [275.0, 219.0, 170.0, 188.0,  86.0, 104.0, 132.0]
range_L = [237.0, 254.0, 300.0, 448.0, 435.0, 296.0, 381.0]

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

    # normalization
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

    print("[R_1, R_2, R_3, R_4, R_5, R_6, R_7]")
    print(R_nor_data)
    print("[L_1, L_2, L_3, L_4, L_5, L_6, L_7]")
    print(L_nor_data)


fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def animate(i):
    pullData = data[0][0]
    dataArray = pullData.split('\n')
    xar = []
    yar = []
    for eachLine in dataArray:
    if len(eachLine)>1:
        x,y = eachLine.split(',')
        xar.append(int(x))
        yar.append(int(y))
    ax1.clear()
    ax1.plot(xar,yar)

ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()

movObj.setStiffnesses('HeadYaw',0.0)
