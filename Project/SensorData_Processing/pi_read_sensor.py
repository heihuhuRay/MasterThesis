#!/usr/bin/env python
# -*- coding: utf-8 -*-
# __date__ = 20180129
# modified by: Ray

import socket
import time
import os
import RPi.GPIO as GPIO
from readADCMultiMCPDriver import *
import numpy as np

PI_IP_via_cable = "169.254.234.160"

NAO_IP_via_cable = "169.254.92.177"

UDP_PORT = 5005
UDP_IP = NAO_IP_via_cable

num_sensor_left = 7
num_sensor_right = 7

right_sensor = [0]*num_sensor_left
left_sensor = [0]*num_sensor_right


while True:
    while True:
        button_pressed = GPIO.input(21)
        if button_pressed == 1:
            print("Bottom pressed for starting..")
            time.sleep(2)
            break
    sock1 = socket.socket(socket.AF_INET, # Internet
                  socket.SOCK_DGRAM) # UDP
    count = 0
    start_time = time.time()
    total_time = 0


    while True:
        # read the analog pin
        for i in range(0,num_sensor_left):
            right_sensor[i]  = readadcA(i)
            print('right sensor')
            print(right_sensor)
        for i in range(0,num_sensor_right):
            left_sensor[i] = readadcB(i)

        sock1.sendto(str([right_sensor,left_sensor]), (UDP_IP, UDP_PORT))

        count = count +1
        time.sleep(0.01)
        total_time = time.time()-start_time

        button_pressed = GPIO.input(21)
        if button_pressed == 0:
            print("Bottom pressed for starting..")
            time.sleep(2)
            break
    break

    print(" nbr. of reading per second:", count/total_time) 



