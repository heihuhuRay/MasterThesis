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

# print sys.argv

"""
 21. RShoulderPitch 	Right shoulder joint front and back (Y)	-119.5 to 119.5 	-2.0857 to 2.0857
 22. RShoulderRoll 	Right shoulder joint right and left (Z)	-76 to 18           -1.3265 to 0.3142
 23. RElbowYaw 	Right shoulder joint twist (X)              -119.5 to 119.5 	-2.0857 to 2.0857
 24. RElbowRoll 	Right elbow joint (Z)                       2 to 88.5           0.0349 to 1.5446
 25. RWristYaw 	Right wrist joint (X)                       -104.5 to 104.5 	-1.8238 to 1.8238
 26. RHand         Right hand 	Open and Close 	Open and Close

"""

NAOIP = "0.0.0.0"
PORT= 9559


######################################
# Connect to the module ALMotionProxy 
movObj = ALProxy("ALMotion",NAOIP,PORT)
movObj.setStiffnesses('HeadYaw',0.99)

# Connect to the module ALMemoryProxy
memProxy = ALProxy("ALMemory",NAOIP,PORT)

movObj.setAngles("HeadYaw", 0,0.6)

time.sleep(2) 


while True:
    
    FrontTactileON   = memProxy.getData('Device/SubDeviceList/Head/Touch/Front/Sensor/Value')
    if FrontTactileON == 1:
        break 

    data = memProxy.getData("WristForceSensor")
    print data
    print data[1][0]
    #print LeftSens,RightSens
    
    movObj.setAngles("HeadYaw", data[1][0]/1000.0,0.99)
    
    #print data2[4]

    #print data2[4]
    #print data2[0]
    #print "--"
    #print data2[1]
    
    time.sleep(0.1)

movObj.setStiffnesses('HeadYaw',0.0)
