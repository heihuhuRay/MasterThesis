#!/usr/bin/env python
# -*- coding: utf-8 -*-
# __date__ = 20180403
# modified by: Ray
import json
import naoqi
import math
import time
import sys
import random
import numpy as np

from naoqi import ALProxy
from naoqi import ALModule
from alpha_value import *
from SetTiming import *
from MLMPCPG import *
from NAOMotor import *


store_data = []
# Connect to the module ALMemoryProxy
memProxy = ALProxy("ALMemory", 'nao.local', 9559)
movObj = ALProxy("ALMotion", 'nao.local', 9559)
TextObj = ALProxy("ALTextToSpeech", 'nao.local', 9559)
# define sensor
wrist_sensor = memProxy.getData("WristForceSensor")
LHandBackSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Back/Sensor/Value')
LHandLeftSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Left/Sensor/Value')
LHandRightSensor = memProxy.getData('Device/SubDeviceList/LHand/Touch/Right/Sensor/Value')
RHandBackSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Back/Sensor/Value')
RHandLeftSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Left/Sensor/Value')
RHandRightSensor = memProxy.getData('Device/SubDeviceList/RHand/Touch/Right/Sensor/Value')

def release_arm_stiffness():
    movObj.setStiffnesses('LArm', 0 * int( not ((LHandBackSensor == 1) or (LHandLeftSensor == 1)  or (LHandRightSensor == 1))))
    movObj.setStiffnesses('RArm', 0 * int( not ((RHandBackSensor == 1) or (RHandLeftSensor == 1)  or (RHandRightSensor == 1))))

def change_alpha(alpha_AnkelRoll, alpha_HipRoll):
    PF_AnkleRoll = PF_Patterns(alpha_AnkelRoll, 0)
    PF_HipRoll = PF_Patterns(alpha_HipRoll, 0)

    myCont[R_ANKLE_ROLL].fSetPatternRG(RG_AnkleRoll)
    myCont[R_ANKLE_ROLL].fSetPatternPF(PF_AnkleRoll)

    myCont[L_ANKLE_ROLL].fSetPatternRG(RG_AnkleRoll)
    myCont[L_ANKLE_ROLL].fSetPatternPF(PF_AnkleRoll)

    myCont[L_HIP_ROLL].fSetPatternRG(RG_HipRoll)
    myCont[L_HIP_ROLL].fSetPatternPF(PF_HipRoll)

    myCont[R_HIP_ROLL].fSetPatternRG(RG_HipRoll)
    myCont[R_HIP_ROLL].fSetPatternPF(PF_HipRoll)


sys.path.append('mylib/naoConnect')
sys.path.append('mylib/MLMPCPG')
sys.path.append('mylib/naoPlot')
sys.path.append('mylib/extratools')
sys.path.append('mylib/NAOKin')
sys.path.append('mylib\NNET\SOM')
sys.path.append('mylib\MotorProgramInCPG')

######################################
model = 'robot'
# model = 'robot'
if model == 'LArm2D':
    model = 1
    from inverseKin import invKin
    from forwardKin import frwKin

    print 'using forward kinematics'

    initPos = np.genfromtxt(file_path + "CurPos.txt", delimiter=",")
    initPos = np.array(initPos)
    # for i in range(involved_joints.shape[0]):
    #   angles[involved_joints[i]] = starting_val[i] * math.pi / 180
elif model == 'robot':
    model = 0
    import NaoConnect

    if NaoConnect.NaoRobotConnect.RealNaoRobot:
        print "RealNaoRobot: ", NaoConnect.NaoRobotConnect.RealNaoRobot[0]

    if NaoConnect.NaoVrepConnect.NaoVrep:
        print "NaoVrep: ", NaoConnect.NaoVrepConnect.NaoVrep[0]

    # if NaoConnect.NaoWebotsConnect.NaoWebots:
    #     print "NaoWebots: ", NaoConnect.NaoQiConnect.NaoWebots[0]

    NAOosON = NaoConnect.NaoRobotConnect.RealNaoRobot or NaoConnect.NaoVrepConnect.NaoVrep or NaoConnect.NaoWebotsConnect.NaoWebots

    print "NAOosON : ", NAOosON
    if NAOosON == []:
        sys.exit("No robot or simulation connected..!")

    if NaoConnect.NaoRobotConnect.RealNaoRobot:
        NaoConnect.NaoRobotConnect.postObj.goToPosture("Stand", 0.4)

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
    # while angleCount <= legOpenAngleInit:
    #     initPos[R_HIP_PITCH] = -1 * angleCount * math.pi / 180.0
    #     initPos[R_ANKLE_PITCH] = angleCount * math.pi / 180.0
    #     initPos[L_HIP_PITCH] = angleCount * math.pi / 180.0
    #     initPos[L_ANKLE_PITCH] = -1 * angleCount * math.pi / 180.0
    #     angleCount = angleCount + 0.2
    #     NaoConnect.NaoSetAngles(initPos)
    #     time.sleep(0.1)

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
        time.sleep(0.015)

    # initPos[L_HIP_ROLL] = -5 * math.pi / 180.0
    # initPos[R_HIP_ROLL] = 5 * math.pi / 180.0

    initPos[L_ANKLE_ROLL] = 0 * math.pi / 180.0
    initPos[R_ANKLE_ROLL] = 0 * math.pi / 180.0

    # initPos[L_KNEE_PITCH] = 30 * math.pi / 180.0
    # initPos[R_KNEE_PITCH] = 30 * math.pi / 180.0
    # initPos[L_ANKLE_PITCH] = -20 * math.pi / 180.0
    # initPos[R_ANKLE_PITCH] = -20 * math.pi / 180.0
    # initPos[L_HIP_PITCH] = -10 * math.pi / 180.0
    # initPos[R_HIP_PITCH] = -10 * math.pi / 180.0
    NaoConnect.NaoSetAngles(initPos)
    print("###############################################################")
    print("########################### mark 1 ############################")
    print("###############################################################")
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

myT = fSetTiming()

myCont = fnewMLMPcpg(number_cpg)

myCont = fSetCPGNet(myCont,'MyNao.txt','MyNaoPsitiveAngle_E_or_F.txt')
time.sleep(1)
plusPloarity  = 1
minusPloarity  = -1
tempCounter = 0

initPos = NaoConnect.NaoGetAngles()
for i in range(0, len(myCont)):
    myCont[i].fUpdateInitPos(initPos[i])


for i in range(0, len(myCont)):
    myCont[i].fUpdateLocomotionNetwork(myT,initPos[i])
print 'Robot is ready to move..!!'
time.sleep(3)

# tm : tau_m change the spped of the action
# the larger the tau_m is the slower the action accomplished
all_joint_tm = 0.3

sigma_s_test = 2
sigma_f_test = 2.5

#Oscillatory pattern
RG_KneePitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_HipPitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_AnklePitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_HipRoll = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_AnkleRoll = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)

release_arm_stiffness()
# Disable Fall Manager 
#TextObj.say('Attention, Fall Manager is Disabled.')
TextObj.say('Please hold my wrist.')
movObj.setFallManagerEnabled(False) # True False
time.sleep(2)


ExtInjCurr = 0
ExtInjCurr1 = 0

initPos = NaoConnect.NaoGetAngles()
for i in range(0, len(myCont)):
    myCont[i].fUpdateInitPos(initPos[i])
    myCont[i].joint.joint_motor_signal =   myCont[i].joint.init_motor_pos




#######################################################################################
###############################      Main Loop    #####################################
#######################################################################################
for I in range(0, 2000):
    startTime = time.time()
    t= I*myT.T
    # inject positive current
    if I == 10:
        myT.T7 = t
        myT.T8 = myT.T7 + myT.signal_pulse_width
        tune_Ss_time_step = I +500
    if t >= myT.T7 and t <= myT.T8:
        ExtInjCurr = 1
        ExtInjCurr1 = -1
        #print "At ",I," current is injected"
    else:
        ExtInjCurr = 0
        ExtInjCurr1 = 0

    #TODO change alpha here
    index = I % 50
    if index == 0:
        alpha_ankel = random.uniform(0, 0.15)
        alpha_hip = random.uniform(0, 0.15)
        change_alpha(alpha_ankel, alpha_hip)
    store_data.append([alpha_ankel, alpha_hip, wrist_sensor])
    print([alpha_ankel, alpha_hip, wrist_sensor])

    for ii in [R_ANKLE_ROLL, R_HIP_ROLL]:
        myCont[ii].RG.F.InjCurrent_value = +1 * (ExtInjCurr) * myCont[
            ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value = -1 * (ExtInjCurr) * myCont[
            ii].RG.E.InjCurrent_MultiplicationFactor

    for ii in [L_HIP_ROLL, L_ANKLE_ROLL]:
        myCont[ii].RG.F.InjCurrent_value = 1 * (ExtInjCurr1) * myCont[
            ii].RG.F.InjCurrent_MultiplicationFactor
        myCont[ii].RG.E.InjCurrent_value = -1 * (ExtInjCurr1) * myCont[
            ii].RG.E.InjCurrent_MultiplicationFactor

    #TODO just update this 4 joints !!!
    for i in [R_ANKLE_ROLL, R_HIP_ROLL,L_HIP_ROLL, L_ANKLE_ROLL]:
        myCont[i].fUpdateLocomotionNetwork(myT, initPos[i])

    for i in range(0, len(myCont)):
        MotorCommand[i]=myCont[i].joint.joint_motor_signal

    NaoConnect.NaoSetAngles(MotorCommand)
    initPos = NaoConnect.NaoGetAngles()

file_name = 'alpha_sensor_data.json'
with open(file_name,'w') as file_object:
    json.dump(store_data, file_object)

#np.save("alpha_sensor_data.npy", store_data)
