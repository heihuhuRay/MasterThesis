# -*- coding: utf-8 -*-
"""
Created on Sun May 28 21:02:20 2017

@author: nasjo
"""

import sys

sys.path.append('mylib/naoConnect')
sys.path.append('mylib/MLMPCPG')
sys.path.append('mylib/naoPlot')
sys.path.append('mylib/extratools')
sys.path.append('mylib/NAOKin')
sys.path.append('mylib\NNET\SOM')
sys.path.append('mylib\MotorProgramInCPG')
sys.path.append('mylib\payam')

import motorProgramClass
# import MotorProgram_Class as motorProgramClass
import naokinematics

import naoplot
import time
import numpy as np
import math
import random
import scipy.spatial.distance as dist

######################################
from SetTiming import *
from MLMPCPG import *
from NAOMotor import *
from random import randint
from myPloting import *
import som2dnet
import matplotlib.pyplot as plt

from CenterOfMass import CoM
from checkList import chkPntValidity

from ezLineCalc import ezLineCalc
from errorCorrectionLine import errorCorrectionLine
######################################
file_path = "C:\\Users\\Payam\\Desktop\\PHtest\\tt\\NaoControlFramework-master\\maincode\\mylib\\payam\\"

model = 'LArm2D'
if model == 'LArm2D':
    model = 1
    from inverseKin import invKin
    from forwardKin import frwKin

    print 'using forward kinematics'

    initPos = np.genfromtxt(file_path + "CurPos.txt", delimiter=",")
    initPos = np.array(initPos)
    # for i in range(involved_joints.shape[0]):
    #     angles[involved_joints[i]] = starting_val[i] * math.pi / 180
elif model == 'robot':
    model = 0
    import NaoConnect

    if NaoConnect.NaoRobotConnect.RealNaoRobot:
        print "RealNaoRobot: ", NaoConnect.NaoRobotConnect.RealNaoRobot[0]

    if NaoConnect.NaoVrepConnect.NaoVrep:
        print "NaoVrep: ", NaoConnect.NaoVrepConnect.NaoVrep[0]

    if NaoConnect.NaoWebotsConnect.NaoWebots:
        print "NaoWebots: ", NaoConnect.NaoQiConnect.NaoWebots[0]

    NAOosON = NaoConnect.NaoRobotConnect.RealNaoRobot or NaoConnect.NaoVrepConnect.NaoVrep or NaoConnect.NaoWebotsConnect.NaoWebots

    print "NAOosON : ",NAOosON
    if NAOosON == []:
        sys.exit("No robot or simulation connected..!")


    if NaoConnect.NaoRobotConnect.RealNaoRobot:
        NaoConnect.NaoRobotConnect.postObj.goToPosture("Crouch",0.8)

    initPos = NaoConnect.NaoGetAngles()

    # move to init position
    # initPos = numpy.ones(26)*0.00
    initPos[L_SHOULDER_PITCH] = 0 * math.pi / 180.0
    initPos[L_SHOULDER_ROLL] = 20 * math.pi / 180.0
    initPos[L_ELBOW_YAW] = 0 * math.pi / 180.0
    initPos[L_ELBOW_ROLL] = -55 * math.pi / 180.0
    initPos[L_WRIST_YAW] = -90 * math.pi / 180.0
    NaoConnect.NaoSetAngles(initPos)
    time.sleep(2)

    print initPos[L_SHOULDER_PITCH:L_WRIST_YAW + 1]

#######################################################
# Joint Index to be used when referring to a particular joint

"""
HEAD_YAW,HEAD_PITCH,L_SHOULDER_PITCH,L_SHOULDER_ROLL,L_ELBOW_YAW,L_ELBOW_ROLL,
L_WRIST_YAW,L_HAND,L_HIP_YAW_PITCH,L_HIP_ROLL,L_HIP_PITCH,L_KNEE_PITCH,L_ANKLE_PITCH,
L_ANKLE_ROLL,R_HIP_YAW_PITCH,R_HIP_ROLL,R_HIP_PITCH,R_KNEE_PITCH,R_ANKLE_PITCH,
R_ANKLE_ROLL,R_SHOULDER_PITCH,R_SHOULDER_ROLL,R_ELBOW_YAW,R_ELBOW_ROLL,
R_WRIST_YAW,R_HAND
"""
#######################################################
print "Before the main loop.. "

# MAIN LOOP

#  4. LShoulderRoll 	Left shoulder joint right and left (Z) 	-18 to 76           -0.3142 to 1.3265
#  5. LElbowYaw 	Left shoulder joint twist (X)               -119.5 to 119.5 	-2.0857 to 2.0857
#  6. LElbowRoll 	Left elbow joint (Z)                        -88.5 to -2         -1.5446 to -0.0349

alphas_GA_lines = np.array(np.loadtxt(
    'C:\\Users\\Payam\\Desktop\\Python-Master\\NaoControlFramework-master\\maincode\\accepted_alphas_45_-45.out'))

envelope = np.genfromtxt(file_path + "Envelope.txt", delimiter=",")

Cx, Cy = CoM(envelope)

LShoulderRoll_min = -0.3142
LShoulderRoll_max = 1.3265

LElbowRoll_min = -1.5446
LElbowRoll_max = -0.0349

######################################
# SOM

# number of som neurons
MAX_CLUSTERS = 40
# first dimension in the 2D network. Second dimenstion = number of som neurons / first dimension
CLUSTER_DIMENSION1 = 5
# dimesnion of input vector, it is also the dimension of weights vector of SOM
INPUT_DIMENSION = 2

# learning rate reduction factor
DECAY_RATE = 0.95
# minimum value of learning rate
MIN_ALPHA = 0.01
# Starting value of learning rate
ALPHA = 0.6

# creat an object "som" neural network
som = som2dnet.SOM_Class1(INPUT_DIMENSION, MAX_CLUSTERS, CLUSTER_DIMENSION1,ALPHA, MIN_ALPHA, DECAY_RATE)
som.w[:,0] = Cx
som.w[:,1] = Cy
som.plot2D()

rand_training_patterns = 10+np.random.rand(500,INPUT_DIMENSION)*8.0

training_patterns = []
############# #########################


ctr = 0

ALL_joints = []
ALL_EndEff = []

Boundaries2D = np.loadtxt('NAOROBOT\workspace2DLArm4Drawing2.out')

# initial alpha for line drawing
# alphas_GA_lines = np.genfromtxt("mylib/payam/alphas_45_-45.txt", delimiter=",")

count = 0
points = np.empty((0, 2))

while count < 4:
    count += 1
    print 'counter = ', count
    point, theta_vec = chkPntValidity(envelope)
    if points.size == 0:
        points = np.append(points, [point], axis=0)
    if np.sum(np.logical_and((points - point == [0, 0])[:, 0], (points - point == [0, 0])[:, 1])) == 0:
        if points.size > 1:
            points = np.append(points, [point], axis=0)

        initPos[L_SHOULDER_ROLL] = theta_vec[0]*math.pi/180
        initPos[L_ELBOW_ROLL] = theta_vec[1]*math.pi/180
        # initPos[L_SHOULDER_ROLL] = 45 * math.pi / 180
        # initPos[L_ELBOW_ROLL] = -45 * math.pi / 180
        if model == 0:
            NaoConnect.NaoSetAngles(initPos)
            time.sleep(2)
        #############################################################################
        # Generate a random point in the workspace with 1 cm margin
        #############################################################################
        # generate random angles for shoulder roll and elbow roll



        Transf = naokinematics.fFK2DLeftArm(initPos[L_SHOULDER_PITCH:L_WRIST_YAW+1])
        End_x = Transf[0,3]
        End_y = Transf[1,3]


        #############################################################################
        # RG patterns
        PL1 = RG_Patterns(5,0.1,1,0.1) # Plateau
        PL2 = RG_Patterns(5,0.1,1,0.2) # Plateau

        #while ():
        col_vec = ['r', 'g', 'b', 'y', 'c', 'm', 'g', 'b']
        jet = plt.get_cmap('jet')
        colors = iter(jet(np.linspace(0, 1, 10)))
        fig_GA = plt.figure(1)
        plt.ion()
        plt.show()


        angle_temp = np.zeros(8)
        initPosOLDSR = initPos[L_SHOULDER_ROLL]
        initPosOLDER = initPos[L_ELBOW_ROLL]
        # initPosOLD = initPos
        for i in range(0, alphas_GA_lines.shape[0]):
            initPos[L_SHOULDER_ROLL] = initPosOLDSR
            initPos[L_ELBOW_ROLL] = initPosOLDER
            print "before:"
            print 'initpos 3', initPos[3], 'initpos 5', initPos[5]
            # PF patterns
            PFPat1 = PF_Patterns(alphas_GA_lines[i,0], 0)
            PFPat2 = PF_Patterns(alphas_GA_lines[i,1], 0)
            # motorProgramClass.MotorProgClass(sigmaS=0,sigmaF=0,TauM=0,Iing=0,Tstart=0,AlphaPF=0,ThetaPF=0)
            LSR_M1 = motorProgramClass.MotorProgClass(L_SHOULDER_ROLL, PL1, 2.0, PFPat1)
            LER_M1 = motorProgramClass.MotorProgClass(L_ELBOW_ROLL, PL1, 2.0, PFPat2)

            allmotorprog = [LSR_M1, LER_M1]
            # allmotorprog = [LSR_M1]

            XX_mat, YY_mat = motorProgramClass.RunMotoProgWFrwKin(allmotorprog, initPos)
            # XX_mat, YY_mat = motorProgramClass.RunMotoProgOnNAO(allmotorprog, initPos, 'LArm2D')
            print 'size XX', len(XX_mat)
            #XX_mat, YY_mat =[[0,0],[1,1]]
            print "after:"
            print 'initpos 3', initPos[3],'initpos 5',  initPos[5]
            # the result of the action (in case of line) is given to ezLineCalc to calculate the angle of the drawn line
            angle_temp[i] = ezLineCalc(XX_mat, YY_mat)[0]
            h_GA = plt.plot(XX_mat, YY_mat, color=next(colors), lw=1)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.draw()
            plt.pause(0.1)

        all_data_angle = angle_temp
        all_data_alpha = alphas_GA_lines
        flag_vec_8lines = np.zeros(8)
        desired_angles = np.array([0, 45, 90, 135, 180, -135, -90, -45])
        angles_mat = angle_temp
        alpha_mat = alphas_GA_lines
        accepted_alphas = np.zeros([8,2])


        while (np.sum(flag_vec_8lines) < 8):
            print flag_vec_8lines
            alpha_mat_new, flag_success, accepted_alphas = errorCorrectionLine(angles_mat, alpha_mat, desired_angles[flag_vec_8lines != np.ones(8)],
                                            all_data_angle, all_data_alpha, accepted_alphas, flag_vec_8lines)
            temp_inx_flag = np.nonzero(flag_vec_8lines != np.ones(8))
            temp_inx_success = np.nonzero(np.logical_and(flag_vec_8lines[flag_vec_8lines != np.ones(8)] != np.ones(len(flag_success)), flag_success == 1))
            flag_vec_8lines[temp_inx_flag[0][temp_inx_success[0]]] = 1
            # flag_vec_8lines[flag_vec_8lines != np.ones(8)] != np.ones(len(flag_success))
            alpha_mat_new = np.array(alpha_mat_new)
            angle_temp = np.zeros(alpha_mat_new.shape[0])
            for i in range(0, alpha_mat_new.shape[0]):
                initPos[L_SHOULDER_ROLL] = initPosOLDSR
                initPos[L_ELBOW_ROLL] = initPosOLDER
                # PF patterns
                PFPat1 = PF_Patterns(alpha_mat_new[i, 0], 0)
                PFPat2 = PF_Patterns(alpha_mat_new[i, 1], 0)
                # motorProgramClass.MotorProgClass(sigmaS=0,sigmaF=0,TauM=0,Iing=0,Tstart=0,AlphaPF=0,ThetaPF=0)
                LSR_M1 = motorProgramClass.MotorProgClass(L_SHOULDER_ROLL, PL1, 2.0, PFPat1)
                LER_M1 = motorProgramClass.MotorProgClass(L_ELBOW_ROLL, PL1, 2.0, PFPat2)

                allmotorprog = [LSR_M1, LER_M1]
                # allmotorprog = [LSR_M1]
                XX_mat, YY_mat = motorProgramClass.RunMotoProgWFrwKin(allmotorprog, initPos)
                # XX_mat, YY_mat = motorProgramClass.RunMotoProgOnNAO(allmotorprog, initPos,'LArm2D')

                angle_temp[i] = ezLineCalc(XX_mat, YY_mat)[0]
            for i in range (0, len(angle_temp)):
                if np.sum(all_data_angle - angle_temp[i] == 0) == 0:
                    all_data_angle = np.append(all_data_angle,angle_temp[i])
                    all_data_alpha = np.append(all_data_alpha, [np.transpose(alpha_mat_new[i,:])], axis=0)
            print all_data_angle
            angles_mat = angle_temp
            alpha_mat = alpha_mat_new

        angle_temp = np.zeros(accepted_alphas.shape[0])
        col_vec = ['r', 'g', 'b', 'y', 'c', 'm', 'g', 'b']
        jet = plt.get_cmap('jet')
        colors = iter(jet(np.linspace(0, 1, 10)))
        fig_Final = plt.figure(2)
        plt.ion()
        plt.show()
        for i in range(0, accepted_alphas.shape[0]):
            initPos[L_SHOULDER_ROLL] = initPosOLDSR
            initPos[L_ELBOW_ROLL] = initPosOLDER
            PFPat1 = PF_Patterns(accepted_alphas[i,0], 0)
            PFPat2 = PF_Patterns(accepted_alphas[i,1], 0)
            # motorProgramClass.MotorProgClass(sigmaS=0,sigmaF=0,TauM=0,Iing=0,Tstart=0,AlphaPF=0,ThetaPF=0)
            LSR_M1 = motorProgramClass.MotorProgClass(L_SHOULDER_ROLL, PL1, 2.0, PFPat1)
            LER_M1 = motorProgramClass.MotorProgClass(L_ELBOW_ROLL, PL1, 2.0, PFPat2)

            allmotorprog = [LSR_M1, LER_M1]
            # allmotorprog = [LSR_M1]
            XX_mat, YY_mat = motorProgramClass.RunMotoProgWFrwKin(allmotorprog, initPos)
            # XX_mat, YY_mat = motorProgramClass.RunMotoProgOnNAO(allmotorprog, initPos,'LArm2D')

            h_GA = plt.plot(XX_mat, YY_mat, color=next(colors), lw=1)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.draw()
            plt.pause(0.1)
            # the result of the action (in case of line) is given to ezLineCalc to calculate the angle of the drawn line
            angle_temp[i] = ezLineCalc(XX_mat, YY_mat)[0]
        """
        
        """

        ALL_joints.append(initPos)
        ALL_EndEff.append(point)


        training_patterns.append([point[0] , point[1]])
        som.train(np.array(training_patterns))
        som.plot2D(np.array(training_patterns))
        # som.plot2D()


if model==0:
    NaoConnect.NaoStop()

    allcom = np.loadtxt('allcomand.out')
    allsen = np.loadtxt('alljsensor.out')
#alliinj = np.loadtxt('alliinj.out')





# ArmTrajX,ArmTrajY = naoplot.fPlot2DLeftArm(All_Command,All_Joints_Sensor,1,'Left Arm','arm_on') # Drawing the arm :arm_off / arm_on 

