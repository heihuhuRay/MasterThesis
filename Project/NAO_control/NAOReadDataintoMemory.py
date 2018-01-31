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

memProxy = ALProxy("ALMemory","localhost",9559)

# for the UDP communication 


PI_IP = "169.254.234.160"

NAO_IP = "169.254.92.177"


NAOIP = "0.0.0.0"
PORT= 9559

tor = []

######################################
# Connect to the module ALMemoryProxy
memProxy = ALProxy("ALMemory",NAOIP,PORT)

time.sleep(2) 

UDP_PORT1 = 5005
UDP_PORT2 = 5006

UDP_IP1 = NAO_IP
UDP_IP2 = PI_IP


#while True:
#    
#    FrontTactileON   = memProxy.getData('Device/SubDeviceList/Head/Touch/Front/Sensor/Value')
#    if FrontTactileON == 1:
#        print "Start reading touch sensors.." 
#        break 


sock1 = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

sock1.bind((UDP_IP1, UDP_PORT1))

#i = 0

#start_time = time.time()

while True:
    #i = i+1
    
    #RearTactileON   = memProxy.getData('Device/SubDeviceList/Head/Touch/Rear/Sensor/Value')
    #if RearTactileON == 1:
    #    break 
    #j=0
        
    data, addr = sock1.recvfrom(1000) # buffer size is 1024 bytes
    
    # receive all packages
    #data, addr = sock1.ioctl(socket.SIO_RCVALL, socket.RCVALL_ON)


    #data, addr = sock1.recvfrom(4) # buffer size is 1024 bytes
    #tor = time.time() - start_time



    #print "received message:", data
    data2 = ast.literal_eval(data)
    #print "my data: ",data2
    memProxy.insertData("WristForceSensor", data2)


    #print "-----"
    
    #print data2[4]

    #print data2[4]
    #print data2[0]
    #print "--"
    #print data2[1]
    
    #time.sleep(0.030)

#print "The number of readings per second: ", i/tor
