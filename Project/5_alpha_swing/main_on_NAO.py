# -*- coding: UTF-8 -*-
from __future__ import print_function
from decimal import Decimal
#!/usr/bin/env python
# __date__ = 20180411
# created by: Ray
# this program should be run on your laptop, not on Pi or NAO


import time
import math
import pandas as pd
import numpy as np
import random
from alpha_value import *
from SetTiming import *
from NAOMotor import *
from MLMPCPG import *
from naoqi import ALProxy
from swing_on_real_Nao import  swing_on_Nao
from swing_on_real_Nao import release_arm_stiffness
from q_learning_data_structure import *

#########################################################
number_cpg = 26

global All_Command
global All_Sensor
global All_FSR, All_cur_out,All_RG_out
global All_PF_out, All_zmp, All_alpha

All_Command=[]
All_Sensor=[]
All_FSR = []
All_cur_out = []
All_RG_out = []
All_PF_out = []
All_zmp = []
All_alpha = []

# tm : tau_m change the spped of the action
# the larger the tau_m is the slower the action accomplished
all_joint_tm = 0.015

sigma_s_test = 2
sigma_f_test = 2.5

# Connect to the module ALMemoryProxy
NAOIP = "192.168.0.110"
PORT= 9559
memProxy = ALProxy("ALMemory", NAOIP, PORT)
movObj = ALProxy("ALMotion", NAOIP, PORT)
TextObj = ALProxy("ALTextToSpeech", NAOIP, PORT)

myCont = fnewMLMPcpg(number_cpg)

myCont = fSetCPGNet(myCont,'MyNao.txt','MyNaoPsitiveAngle_E_or_F.txt')

# define sensor
LHandBackSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Back/Sensor/Value')
LHandLeftSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Left/Sensor/Value')
LHandRightSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Right/Sensor/Value')
RHandBackSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Back/Sensor/Value')
RHandLeftSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Left/Sensor/Value')
RHandRightSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Right/Sensor/Value')

#Oscillatory pattern
RG_KneePitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_HipPitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_AnklePitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_HipRoll = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_AnkleRoll = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
#########################################################


PORT = 9559
robotIP = "192.168.0.110"

try:
    memProxy = ALProxy("ALMemory", robotIP, PORT)
    TextObj = ALProxy("ALTextToSpeech", robotIP, PORT)
    movObj = ALProxy("ALMotion", robotIP, PORT)
except Exception, e:
    print("Could not create proxy to ALMemory")
    print("Error was: ", e)





def update_Q_table(current_state_index, next_state_index, action, reward):
    print('reward', reward)
    current_state = state_list[current_state_index]
    next_state = state_list[next_state_index]
    q_current = Q_table.loc[current_state, action]
    
    if current_state_index != 6: # if current state is not terminal state
        # the very key point of Q-learning, how q_value is updated
        q_new = gamma * Q_table.loc[next_state, :].max()
    else:
        q_new = 0  # next state is terminal
    Q_table.loc[current_state, action] += lr * (reward + q_new - q_current)  # update


# def check_reward(next_state_index):
#     reward = reward_list[next_state_index]
#     if_done = is_next_state_done[next_state_index]
#     return reward, if_done

def check_reward(next_state_index, mean_loop_sensor):
    #TODO calc the reward
    reward = 100 - mean_loop_sensor/100
    #reward = reward_list[next_state_index]
    if_done = is_next_state_done[next_state_index]
    return reward, if_done

def train():
    #alpha_hip = -100
    # stand
    import NaoConnect
    if NaoConnect.NaoRobotConnect.RealNaoRobot:
        print("RealNaoRobot: ", NaoConnect.NaoRobotConnect.RealNaoRobot[0])
    NAOosON = NaoConnect.NaoRobotConnect.RealNaoRobot or NaoConnect.NaoVrepConnect.NaoVrep or NaoConnect.NaoWebotsConnect.NaoWebots
    print("NAOosON : ", NAOosON)
    if NAOosON == []:
        sys.exit("No robot or simulation connected..!")
    if NaoConnect.NaoRobotConnect.RealNaoRobot:
        NaoConnect.NaoRobotConnect.postObj.goToPosture("Stand", 0.5)
    
    initPos = NaoConnect.NaoGetAngles()

    # move to init position
    # initPos = numpy.ones(26)*0.00
    initPos[L_HIP_ROLL] = 0 * math.pi / 180.0
    initPos[R_HIP_ROLL] = 0 * math.pi / 180.0
    initPos[L_ANKLE_PITCH] = 0 * math.pi / 180.0
    initPos[R_ANKLE_PITCH] = 0 * math.pi / 180.0
    initPos[R_HIP_YAW_PITCH] = 0 * math.pi / 180.0
    initPos[L_HIP_YAW_PITCH] = 0 * math.pi / 180.0
    initPos[L_SHOULDER_PITCH] = 90 * math.pi / 180.0
    initPos[R_SHOULDER_PITCH] = 90 * math.pi / 180.0
    NaoConnect.NaoSetAngles(initPos)
    time.sleep(1)

    #print initPos[L_SHOULDER_PITCH:L_WRIST_YAW + 1]

    legOpenAngleInit = 5
    angleCount = 0.0


    hip_pitch_angle = 20
    knee_pitch_angle = 30
    ankle_pitch_angle = 20
    # NaoConnect.NaoSetAngles(initPos)
    while angleCount <= 30:
        initPos[L_KNEE_PITCH] = angleCount * math.pi / 180.0
        initPos[R_KNEE_PITCH] = angleCount * math.pi / 180.0
        initPos[L_ANKLE_PITCH] = -0.66*angleCount * math.pi / 180.0
        initPos[R_ANKLE_PITCH] = -0.66*angleCount * math.pi / 180.0
        initPos[L_HIP_PITCH] = -0.33*angleCount * math.pi / 180.0
        initPos[R_HIP_PITCH] = -0.33*angleCount * math.pi / 180.0
        angleCount = angleCount + 1
        NaoConnect.NaoSetAngles(initPos)
        #time.sleep(0.05)

    initPos[L_HIP_ROLL] = -5 * math.pi / 180.0
    initPos[R_HIP_ROLL] = 5 * math.pi / 180.0

    initPos[L_ANKLE_ROLL] = 0 * math.pi / 180.0
    initPos[R_ANKLE_ROLL] = 0 * math.pi / 180.0

    initPos[L_KNEE_PITCH] = 30 * math.pi / 180.0
    initPos[R_KNEE_PITCH] = 30 * math.pi / 180.0
    initPos[L_ANKLE_PITCH] = -20 * math.pi / 180.0
    initPos[R_ANKLE_PITCH] = -20 * math.pi / 180.0
    initPos[L_HIP_PITCH] = -10 * math.pi / 180.0
    initPos[R_HIP_PITCH] = -10 * math.pi / 180.0
    
    NaoConnect.NaoSetAngles(initPos)
    # Disable Fall Manager
    release_arm_stiffness() 
    TextObj.say('Please hold my wrist.')
    TextObj.say('Attention, Fall Manager is Disabled.')
    movObj.setFallManagerEnabled(False) # True False
    time.sleep(4)
    '''run 100 experiments'''
    for episode in range(10):
        release_arm_stiffness()
        print()
        print('------------------------------------')
        print('-----------episode No.', episode, '-----------')
        print('------------------------------------')
        print('Q_table', Q_table)

        sensor_data = memProxy.getData("WristForceSensor")
        print('WristForceSensor first print', sensor_data)

        # pick a random state from the state list
        current_state_index = random.randint(0, num_state)
        # check if  state_dict is empty
        print('---------------state dict----------------')
        pprint.pprint(state_dict)
        alpha_groups = state_dict[current_state_index]
        '''for each experiment'''
        k = 0
        while True:
            
            k += 1
            # if the random init is the terminal state, then break
            # because in this situation, the q_value should update
            # find a method to evaluate the boundry condition, like the robot fall down or not
            # maybe use the sensor value, if it exceeds a threshold
            # compare the 2 sensor data, 
            print('WristForceSensor 2nd print', sensor_data)

            if (sensor_data > 1000):
                reward = -10
                # update Q-table
                update_Q_table()
                break
            if (sensor_data < 300):
                reward = 10
                break
            print()
            print('############## k =', k, '###############')
            # 1, choose action based on current_state
            action_groups = []
            action_groups = choose_action(current_state_index)#after action, should calc state rather than alpha

            # 2, take action, calc next_state
            #print('current_state_index = ', current_state_index)
            next_state_index, new_alpha_groups = get_next_state_and_new_alpha(current_state_index, action_groups)

            # print('alpha_hip == next_state ==', alpha_hip)
            #TODO not run on NAO for now
            #swing_in_Vrep(alpha_hip) # execute the new alpha_hip in Vrep
            mean_loop_sensor = swing_on_Nao(alpha_hip, 250)
            print('mean_loop_sensor', mean_loop_sensor)
            #time.sleep(5)

            # 3, check reward and if_done
            #print('  next_state_index =  ', next_state_index)
            reward, if_done = check_reward(next_state_index, mean_loop_sensor)
            print('reward', reward)
            print('if_done', if_done)

            # 4, '''update Q-table'''
            update_Q_table(current_state_index, next_state_index, action, reward)
            print('Q_table', Q_table)
            #print('next_state_index ____2', next_state_index)
            current_state_index = next_state_index
            # break while loop when end of this episode
            if if_done:
                break

    # end of game
    print('game over')
    


if __name__ == "__main__":
    train()