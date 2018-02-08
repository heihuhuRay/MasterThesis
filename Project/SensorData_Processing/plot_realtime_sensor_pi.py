#!/usr/bin/env python
#!/usr/bin/env python # -*- coding: utf-8 -*- 
import numpy as np
import matplotlib.pyplot as plt
import socket
import time
import os
import RPi.GPIO as GPIO
from readADCMultiMCPDriver import *
import numpy as np
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation


num_sensor_left = 7
num_sensor_right = 7

right_sensor = [0]*num_sensor_left
left_sensor = [0]*num_sensor_right



plt.xlim(0,20)
plt.ylim(480, 530)
plt.ion()
y = []
i = 0


while True:
    # read the analog pin
    # for i in range(0,num_sensor_left):
    #     right_sensor[i]  = readadcA(i)
    #     print('right sensor')
    #     print(right_sensor)
    # for i in range(0,num_sensor_right):
    #     left_sensor[i] = readadcB(i)
    right_sensor[0]  = readadcA(0)
    temp = right_sensor[0]
    i += 1
    y.append(temp)
    if i>20:
        plt.xlim(i-20,i)
    plt.plot(y)
    plt.pause(0.005)
