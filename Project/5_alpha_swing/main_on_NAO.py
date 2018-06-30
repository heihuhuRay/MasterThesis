# -*- coding: UTF-8 -*-
from __future__ import print_function

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




gamma = 0.9   #reward_decay=0.9
lr = 0.01     #learning_rate=0.01
epsilon = 0.9 #e_greedy=0.9

# the state is the max_angle_X
action_list = [ 'alpha_hip_roll++', 'alpha_hip_roll--', 'alpha_ankle_roll++', 'alpha_ankle_roll--', 
                'alpha_hip_pitch++', 'alpha_hip_pitch--', 'alpha_ankle_pitch++', 'alpha_ankle_pitch--']
# seperate the roll parameters and pitch parameters
#action_list = ['alpha_hip_pitch++', 'alpha_hip_pitch--', 'alpha_ankle_pitch++', 'alpha_ankle_pitch--']


state_sum = len(state_list) - 1
print('state_sum', state_sum)

def get_next_state_index(current_state_index, action):
    '''
    input   current_state_index: 0, 1, 2, 3, 4, 5, 6
            action: string
    output  next_state_index: int [0,6]
    '''
    if (current_state_index == 0) or (current_state_index == 6):
        # if the random init is the terminal state, then break
        # because in this situation, the q_value should update
        raise('Error: current_state is terminal state, check input source')
    else:
        if action == 'alpha_hip++':
            if current_state_index <= (state_sum-1):
                next_state_index = current_state_index+1
            if current_state_index == state_sum: # the last state
                next_state_index = current_state_index
        if action == 'alpha_hip--':
            if current_state_index >= 1:
                next_state_index = current_state_index-1
            if current_state_index == 0:
                next_state_index = current_state_index
    return next_state_index

# No need to calc alpha, just check table
def execute_action(alpha_hip, action):
    if action == 'alpha_hip++':
        alpha_hip += 0.01
    if action == 'alpha_hip--':
        alpha_hip -= 0.01
    return alpha_hip

def choose_action(current_state):
    if np.random.uniform() < epsilon:
        state_action = Q_table.loc[current_state, :]
        state_action = state_action.reindex(np.random.permutation(state_action.index))     # some actions have same value
        action = state_action.idxmax()  # choose best action
    else:
        action = np.random.choice(action_list) # choose random action from action_list
    return action

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
        # sensor_data = memProxy.getData("WristForceSensor")
        # print('WristForceSensor', sensor_data)
        # pick a random state from the state list
        current_state_index = random.randint(0, state_sum)
        current_state = state_list[current_state_index]
        alpha_hip = current_state
        '''for each experiment'''
        k = 0
        while True:
            
            k += 1
            # if the random init is the terminal state, then break
            # because in this situation, the q_value should update
            # find a method to evaluate the boundry condition, like the robot fall down or not
            # maybe use the sensor value, if it exceeds a threshold
            if (current_state_index == 0) or (current_state_index == 6):
                
                break
            print()
            print('############## k =', k, '###############')
            # 1, choose action based on current_state
            action = choose_action(current_state)#after action, should calc state rather than alpha

            # 2, take action, calc next_state
            #print('current_state_index = ', current_state_index)
            next_state_index = get_next_state_index(current_state_index, action)

            alpha_hip = state_list[next_state_index]
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