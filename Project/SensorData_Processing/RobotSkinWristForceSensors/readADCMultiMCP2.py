
import socket
import time
import os
import RPi.GPIO as GPIO
from readADCMultiMCPDriver import *
import numpy as np

PI_IP = "169.254.234.160"

NAO_IP = "169.254.92.177"

UDP_PORT = 5005
UDP_IP = NAO_IP

NbrSensorLeft = 7
NbrSensorRight = 7

LeftSens = [0]*NbrSensorLeft
RightSens = [0]*NbrSensorRight


while True:
        while True:
                buttonpressede = GPIO.input(21)
                if buttonpressede == 1:
                        print "Bottom pressed for starting.."
                        time.sleep(2)
                        break
        sock1 = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        ctr = 0
        start_time = time.time()
        tor = 0


        while True:
                # read the analog pin
                for i in range(0,NbrSensorLeft):
                        LeftSens[i]  = readadcA(i)
                for i in range(0,NbrSensorRight):
                        RightSens[i] = readadcB(i)
                        


                sock1.sendto(str([LeftSens,RightSens]), (UDP_IP, UDP_PORT))
                        

                ctr = ctr +1
                time.sleep(0.01)
                tor = time.time()-start_time


                buttonpressede = GPIO.input(21)
                if buttonpressede == 0:
                        print "Bottom pressed for starting.."
                        time.sleep(2)
                        break

        print " nbr. of reading per second:", ctr/tor 



