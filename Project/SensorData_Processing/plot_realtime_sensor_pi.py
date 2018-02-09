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
min_R = [491, 670, 708, 685, 769, 714, 416]
min_L = [287, 530, 645, 431, 377, 473, 554]

max_R = [766, 889, 878, 873, 855, 818, 548]
max_L = [524, 784, 945, 879, 812, 769, 935]

#Observation data
sti_L = [355, 665, 734, 497, 699, 560, 580]
sti_R = [598, 722, 725, 729, 798, 733, 469]
'''
sti_L = [350, 660, 730, 490, 690, 550, 570]
sti_R = [590, 720, 720, 720, 790, 730, 460]
'''
range_R = []
range_L = []
##################################################################################

for i in range(7):
    r_R = max_R[i] - sti_R[i]
    r_L = max_L[i] - sti_L[i]
    range_R.append(r_R)
    range_L.append(r_L)
print(range_R)
print(range_L)

num_sensor_left = 7
num_sensor_right = 7

right_sensor = [0]*num_sensor_left
left_sensor = [0]*num_sensor_right



plt.xlim(0,20)
plt.ylim(0, 1)
plt.ion()
y = []
i = 0


while True:
    # read the analog pin
    for i in range(0,num_sensor_left):
        right_sensor[i]  = readadcA(i)

    for i in range(0,num_sensor_right):
        left_sensor[i] = readadcB(i)
    # assign sensor value to data
    data = [right_sensor, left_sensor]

    R_nor_data = [  (data[0][0] - sti_R[0])/range_R[0],
            (data[0][1] - sti_R[1])/range_R[1],
            (data[0][2] - sti_R[2])/range_R[2],
            (data[0][3] - sti_R[3])/range_R[3],
            (data[0][4] - sti_R[4])/range_R[4],
            (data[0][5] - sti_R[5])/range_R[5],
            (data[0][6] - sti_R[6])/range_R[6],
        ]

    L_nor_data = [  (data[1][0] - sti_L[0])/range_L[0],
            (data[1][1] - sti_L[1])/range_L[1],
            (data[1][2] - sti_L[2])/range_L[2],
            (data[1][3] - sti_L[3])/range_L[3],
            (data[1][4] - sti_L[4])/range_L[4],
            (data[1][5] - sti_L[5])/range_L[5],
            (data[1][6] - sti_L[6])/range_L[6],
        ]

    temp = R_nor_data[0]
    i += 1
    y.append(temp)
    if i>20:
        plt.xlim(i-20,i)
    plt.plot(y)
    plt.pause(0.005)
