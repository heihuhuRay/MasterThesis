#!/usr/bin/env python
# -*- coding: utf-8 -*-
# __date__ = 20180403
# modified by: Ray
import json
import naoqi
import math
import time
import sys
import socket
import threading
import random
import numpy as np
import NaoConnect

from naoqi import ALProxy
from naoqi import ALModule
from alpha_value import *
from SetTiming import *
from MLMPCPG import *
from NAOMotor import *

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



if NaoConnect.NaoRobotConnect.RealNaoRobot:
    print "RealNaoRobot: ", NaoConnect.NaoRobotConnect.RealNaoRobot[0]

# if NaoConnect.NaoVrepConnect.NaoVrep:
#     print "NaoVrep: ", NaoConnect.NaoVrepConnect.NaoVrep[0]

# if NaoConnect.NaoWebotsConnect.NaoWebots:
#     print "NaoWebots: ", NaoConnect.NaoQiConnect.NaoWebots[0]

NAOosON = NaoConnect.NaoRobotConnect.RealNaoRobot or NaoConnect.NaoVrepConnect.NaoVrep or NaoConnect.NaoWebotsConnect.NaoWebots

print "NAOosON : ", NAOosON
if NAOosON == []:
    sys.exit("No robot or simulation connected..!")

#----------------------------------------------------------------------------------------------------

# define sensor
LHandBackSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Back/Sensor/Value')
LHandLeftSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Left/Sensor/Value')
LHandRightSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Right/Sensor/Value')
RHandBackSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Back/Sensor/Value')
RHandLeftSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Left/Sensor/Value')
RHandRightSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Right/Sensor/Value')

#Oscillatory pattern
RG_AnkleRoll = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_KneePitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_HipRoll = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_HipPitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)



myCont[L_ANKLE_ROLL].fSetPatternRG(RG_AnkleRoll)
myCont[R_ANKLE_ROLL].fSetPatternRG(RG_AnkleRoll)

myCont[L_KNEE_PITCH].fSetPatternRG(RG_KneePitch)
myCont[R_KNEE_PITCH].fSetPatternRG(RG_KneePitch)

myCont[L_HIP_ROLL].fSetPatternRG(RG_HipRoll)
myCont[R_HIP_ROLL].fSetPatternRG(RG_HipRoll)

myCont[L_HIP_PITCH].fSetPatternRG(RG_HipPitch)
myCont[R_HIP_PITCH].fSetPatternRG(RG_HipPitch)


'''
# PF layer define the alpha here
myCont[L_ANKLE_ROLL].fSetPatternPF(PF_AnkleRoll)
myCont[R_ANKLE_ROLL].fSetPatternPF(PF_AnkleRoll)

myCont[L_KNEE_PITCH].fSetPatternPF(PF_KneePitch)
myCont[R_KNEE_PITCH].fSetPatternPF(PF_KneePitch)

myCont[L_HIP_ROLL].fSetPatternPF(PF_HipRoll)
myCont[R_HIP_ROLL].fSetPatternPF(PF_HipRoll)

myCont[L_HIP_PITCH].fSetPatternPF(PF_HipPitch)
myCont[R_HIP_PITCH].fSetPatternPF(PF_HipPitch)
'''

def release_arm_stiffness():
    movObj.setStiffnesses('LArm', 0 * int( not ((LHandBackSensor == 1) or (LHandLeftSensor == 1)  or (LHandRightSensor == 1))))
    movObj.setStiffnesses('RArm', 0 * int( not ((RHandBackSensor == 1) or (RHandLeftSensor == 1)  or (RHandRightSensor == 1))))

def change_alpha(alpha_groups):
    a_ankle_roll = alpha_groups[0]
    a_knee_pitch = alpha_groups[1]
    a_hip_roll   = alpha_groups[2]
    a_hip_pitch  = alpha_groups[3]

    PF_AnkleRoll = PF_Patterns(a_ankle_roll, 0)
    PF_KneePitch = PF_Patterns(a_knee_pitch, 0)
    PF_HipRoll   = PF_Patterns(a_hip_roll, 0)
    PF_HipPitch  = PF_Patterns(a_hip_pitch, 0)

    # PF layer define the alpha here
    myCont[L_ANKLE_ROLL].fSetPatternPF(PF_AnkleRoll)
    myCont[R_ANKLE_ROLL].fSetPatternPF(PF_AnkleRoll)

    myCont[L_KNEE_PITCH].fSetPatternPF(PF_KneePitch)
    myCont[R_KNEE_PITCH].fSetPatternPF(PF_KneePitch)

    myCont[L_HIP_ROLL].fSetPatternPF(PF_HipRoll)
    myCont[R_HIP_ROLL].fSetPatternPF(PF_HipRoll)

    myCont[L_HIP_PITCH].fSetPatternPF(PF_HipPitch)
    myCont[R_HIP_PITCH].fSetPatternPF(PF_HipPitch)




def swing_on_Nao(alpha_groups, looptimes):
    #release_arm_stiffness()
    sum_loop_sensor = 0
    print('alpha_groups is:', alpha_groups)
    store_data = []
    ######################################
    print("###############################################################")
    print("########################### mark 1 ############################")
    print("###############################################################")

    ExtInjCurr = 0
    ExtInjCurr1 = 0

    initPos = NaoConnect.NaoGetAngles()
    for i in range(0, len(myCont)):
        myCont[i].fUpdateInitPos(initPos[i])
        myCont[i].joint.joint_motor_signal = myCont[i].joint.init_motor_pos

    #######################################################################################
    ###############################      Main Loop    #####################################
    #######################################################################################
    TextObj.say('My current alpha hip is'+str(alpha_hip))
    for I in range(0, looptimes):
        # read sensor data every loop
        wrist_sensor = memProxy.getData("WristForceSensor")
        sum_loop_sensor = sum(wrist_sensor[0]) + sum(wrist_sensor[1]) + sum_loop_sensor
        #print('sum(wrist_sensor) = ', sum_loop_sensor)

        startTime = time.time()
        t= I*myT.T

        # # inject positive current
        # if I == 10:
        #     myT.T7 = t
        #     myT.T8 = myT.T7 + myT.signal_pulse_width
        #     tune_Ss_time_step = I +500
        # if t >= myT.T7 and t <= myT.T8:
        #     ExtInjCurr = 1; ExtInjCurr1 = -1
        # else:
        #     ExtInjCurr = 0; ExtInjCurr1 = 0

        if t>= myT.T1 and t < myT.T2:
            ExtInjCurr = 1
        #elif t >= myT.T3 and t < myT.T4:
        #    ExtInjCurr = 4.5
        #elif t >= myT.T5 and t < myT.T6:    
        #    ExtInjCurr = -4.5
        else: 
            ExtInjCurr = 0
            
        if t>= myT.T3 and t < myT.T4:
            ExtInjCurr2 = -1
        #elif t >= myT.T3 and t < myT.T4:
        #    ExtInjCurr = 4.5
        #elif t >= myT.T5 and t < myT.T6:    
        #    ExtInjCurr = -4.5
        else: 
            ExtInjCurr2 = 0

        #if index == 0:
            # alpha_ankel = random.uniform(0, 0.15)
            # alpha_hip = random.uniform(0, 0.15)

        for ii in [R_ANKLE_ROLL, R_HIP_ROLL]:
            myCont[ii].RG.F.InjCurrent_value = +1 * (ExtInjCurr) * myCont[ii].RG.F.InjCurrent_MultiplicationFactor
            myCont[ii].RG.E.InjCurrent_value = -1 * (ExtInjCurr) * myCont[ii].RG.E.InjCurrent_MultiplicationFactor
        for ii in [L_HIP_ROLL, L_ANKLE_ROLL]:
            myCont[ii].RG.F.InjCurrent_value = -1 * (ExtInjCurr) * myCont[ii].RG.F.InjCurrent_MultiplicationFactor
            myCont[ii].RG.E.InjCurrent_value = +1 * (ExtInjCurr) * myCont[ii].RG.E.InjCurrent_MultiplicationFactor

        for ii in [R_HIP_PITCH, R_KNEE_PITCH, L_ANKLE_PITCH]:
            myCont[ii].RG.F.InjCurrent_value = +1*ExtInjCurr2* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
            myCont[ii].RG.E.InjCurrent_value = -1*ExtInjCurr2* myCont[ii].RG.E.InjCurrent_MultiplicationFactor

        for ii in [L_HIP_PITCH, L_KNEE_PITCH, R_ANKLE_PITCH]:
            myCont[ii].RG.F.InjCurrent_value = -1*ExtInjCurr2* myCont[ii].RG.F.InjCurrent_MultiplicationFactor
            myCont[ii].RG.E.InjCurrent_value = +1*ExtInjCurr2* myCont[ii].RG.E.InjCurrent_MultiplicationFactor

        for i in [L_HIP_ROLL, L_ANKLE_ROLL,R_HIP_ROLL, R_ANKLE_ROLL,R_HIP_PITCH, R_KNEE_PITCH, L_ANKLE_PITCH,L_HIP_PITCH, L_KNEE_PITCH, R_ANKLE_PITCH]:
            myCont[i].fUpdateLocomotionNetwork(myT, initPos[i])

        for i in range(0, len(myCont)):
            MotorCommand[i] = myCont[i].joint.joint_motor_signal

        NaoConnect.NaoSetAngles(MotorCommand)
        initPos = NaoConnect.NaoGetAngles()

    mean_loop_sensor = sum_loop_sensor/looptimes
    return mean_loop_sensor


def main():
    swing_on_Nao([0.042, 0.011, 0.02, 0.022], 1000)

if __name__ == '__main__':
    main()