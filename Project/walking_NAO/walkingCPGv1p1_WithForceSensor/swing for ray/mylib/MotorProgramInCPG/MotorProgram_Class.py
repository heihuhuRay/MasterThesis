# -*- coding: utf-8 -*-
"""
Created on Sun Jun 25 23:57:14 2017

@author: nasjo
"""

import sys

sys.path.append('../../mylib/naoConnect')
sys.path.append('../../mylib/MLMPCPG')
sys.path.append('../../mylib/naoPlot')
sys.path.append('../../mylib/extratools')
sys.path.append('../../mylib/NAOKin')
sys.path.append('../../mylib\NNET\SOM')
sys.path.append('../../mylib\payam')

import naokinematics
import naoplot
import time
import numpy as np
import random
import scipy.spatial.distance as dist
import math
######################################
from SetTiming import *
from MLMPCPG import *
from NAOMotor import *
from random import randint
from myPloting import *
import som2dnet
import matplotlib.pyplot as plt

import copy
######################################


class MotorProgClass:
    def __init__(self, JointIndex, RGpat, Tstart, PFpat):
        # def __init__(self,JointIndex,sigmaS=0,sigmaF=0,TauM=0,Iing=0,Tstart=0,AlphaPF=0,ThetaPF=0):

        self.RGpattern = RGpat
        self.PFpattern = PFpat

        self.Jindex = JointIndex

        self.Tstart = Tstart
        self.Tend = self.Tstart + 0.200  # [ms]

    def fPrintMotorProg(self):
        print '---------------------'
        print 'Joint index= ', self.Jindex, \
            '\nsigmaS=', self.RGpattern.sigma_s, \
            '\nsigmaF=', self.RGpattern.sigma_f, \
            '\nTauM=', self.RGpattern.TAU_M, \
            '\nIinj=', self.RGpattern.InjCurrentMultiplicationFactor, \
            '\nTstart=', self.Tstart, \
            '\nTend=', self.Tend, \
            '\nAlphaPF=', self.PFpattern.alpha, \
            '\nThetaPF=', self.PFpattern.theta
        return


#########################################
global myCont, myT
global All_Command
global All_Joints_Sensor
global All_Iinj


def RunMotoProgOnNAO(MProg, initPos, model):
    if model == 'LArm2D':
        model = 1
        from inverseKin import invKin
        from forwardKin import frwKin
    elif model == 'robot':
        model=0
        import NaoConnect

    # angles = initPos[:]
    angles = copy.deepcopy(initPos)
    All_Command = []
    All_Joints_Sensor = []
    All_Iinj = []
    XX_mat = []
    YY_mat = []
    #######################################################
    number_cpg = 26

    myT = fSetTiming()
    myCont = None
    # Creat list of CPG objects
    myCont = fnewMLMPcpg(number_cpg)
    # Inisiate the CPG list with NAO robot data
    myCont = fSetCPGNet(myCont, 'NAOROBOT\MyNao.txt', 'NAOROBOT\MyNaoPsitiveAngle_E_or_F.txt')
    #######################################################
    # List of joinnts
    JointsList = []
    for i in range(len(MProg)):
        if (MProg[i].Jindex in JointsList):
            continue
        else:
            All_Iinj.append([])
            # print "All_Iinj = ", All_Iinj
            JointsList.append(MProg[i].Jindex)
    # print "JointsList ", JointsList
    #############################
    # Update all joints CPG, it is important to update all joints
    # at least one time, otherwise, non used joints will be set to
    # the defult init posiotion in the CPG which is 0

    for i in range(0, len(myCont)):
        myCont[i].fUpdateInitPos(angles[i])
        #print "fUpdateInitPos for joint ", i, " to ", initPos[i]

    for i in JointsList:
        myCont[i].fUpdateLocomotionNetwork(myT, angles[i])

    # NaoConnect.NaoSetAngles(initPos)

    #for i in range(len(MProg)):
        # print each motor program
        # MProg[i].fPrintMotorProg()
        # print the starting time for each motor program
        #print 'a pattern on joint', MProg[i].Jindex, ' at ', MProg[i].Tstart

    Tcrossed = np.zeros(shape=(len(MProg)))
    InjCurrent = np.zeros(shape=(len(MProg)))
    #print 'Tcrossed = ', Tcrossed

    # print "Before MAIN LOOP "

    # MAIN LOOP
    I = 0
    time1 = time.time()
    tt = time1

    # time.sleep(5)

    # return (1,1)

    # myCont[L_SHOULDER_ROLL].fSetPatternRG(MProg[0].RGpattern) # PatternPL1 PatternOsc3 PatternAosc PatternQU

    # (FsrLeft,FsrRight,robPos,robOrient,HeadTouch,HandTouchLeft,HandTouchRight) = NaoConnect.NaoGetSensors()
    # angles = NaoConnect.NaoGetAngles()
    # angles = initPos
    # print 'INIT angles [3]: ', angles[3], ' angles [5]: ', angles[5]
    final_time = 8.0
    # for i in range(0, len(JointsList)):
        # All_Iinj[i] = np.zeros(final_time / myT.T)
    while ((((time.time() - tt) < final_time) and model == 0) or ((I < final_time/myT.T) and model == 1)):
        I += 1
        t = I * myT.T
        # print t

        """
        if((t>4)and(t<4.2)):
            myCont[L_SHOULDER_ROLL].RG.F.InjCurrent_value=1
            myCont[L_SHOULDER_ROLL].RG.E.InjCurrent_value=-1
        else:
            myCont[L_SHOULDER_ROLL].RG.F.InjCurrent_value=0
            myCont[L_SHOULDER_ROLL].RG.E.InjCurrent_value=0
        """

        for i in range(len(MProg)):
            # print 'compare ', MProg[i].Tstart ,' to ', (time1-tt) , 'Tcross flag: ', Tcrossed[i]
            if ((MProg[i].Tstart < t) and (Tcrossed[i] == 0)):  # Executed only once for each motor program
                # Set the motor program patrameters for joint i
                # print " inside if(( MProg[i].Tstart < t ) and (Tcrossed[i]==0)): ", MProg[i].Jindex

                myCont[MProg[i].Jindex].fSetPatternPF(MProg[i].PFpattern)
                myCont[MProg[i].Jindex].fSetPatternRG(MProg[i].RGpattern)

                myCont[MProg[i].Jindex].RG.E.V = 0
                myCont[MProg[i].Jindex].RG.E.q = 0
                myCont[MProg[i].Jindex].RG.E.out = 0

                myCont[MProg[i].Jindex].RG.F.V = 0
                myCont[MProg[i].Jindex].RG.F.q = 0
                myCont[MProg[i].Jindex].RG.F.out = 0

                Tcrossed[i] = 1.0
                # if model == 0:
                #
                #     # print 'cross at ', time1 - tt
                # elif model == 1:
                #
                #     # print 'cross at ', I
                # update init position for joint i
                # print 'angles[i] is ', angles[MProg[i].Jindex]

                myCont[MProg[i].Jindex].fUpdateInitPos(angles[MProg[i].Jindex])
                # print 'new angles [3]: ', angles[3]*180.0/math.pi, ' new angles [5]: ', angles[5]*180.0/math.pi
                # initPos[i] = angles[i]
            myCont[MProg[i].Jindex].RG.F.InjCurrent_value = 0.0
            myCont[MProg[i].Jindex].RG.E.InjCurrent_value = 0.0

        for i in range(len(MProg)):

            if ((MProg[i].Tstart < t) and (MProg[i].Tend > t)):
                # print "set ", MProg[i].Jindex, " current "
                myCont[MProg[i].Jindex].RG.F.InjCurrent_value = +1 * MProg[i].RGpattern.InjCurrentMultiplicationFactor
                myCont[MProg[i].Jindex].RG.E.InjCurrent_value = -1 * MProg[i].RGpattern.InjCurrentMultiplicationFactor

                # myCont[MProg[i].Jindex].RG.F.InjCurrent_value = -1* myCont[MProg[i].Jindex].RG.F.InjCurrent_MultiplicationFactor
                # myCont[MProg[i].Jindex].RG.E.InjCurrent_value = +1* myCont[MProg[i].Jindex].RG.E.InjCurrent_MultiplicationFactor
                # All_Iinj[JointsList.index(MProg[i].Jindex)][I-1] = All_Iinj[JointsList.index(MProg[i].Jindex)][I-1] + myCont[MProg[i].Jindex].RG.F.InjCurrent_value

            # All_Iinj[JointsList.index(MProg[i].Jindex)].append(myCont[MProg[i].Jindex].RG.F.InjCurrent_value)
            # All_Iinj.append(myCont[MProg[i].Jindex].RG.F.InjCurrent_value)
        # All_Iinj.append(myCont[L_SHOULDER_ROLL].RG.F.InjCurrent_value)

        #####
        #####
        # for i in JointsList:
        #    myCont[i].fUpdateLocomotionNetworkSN(angles[i])

        for i in JointsList:
            myCont[i].fUpdateLocomotionNetwork(myT, angles[i])

        # MotorCommand = initPos
        # for i in range(0, len(myCont)):
        for i in range(0, len(myCont)):
            MotorCommand[i] = myCont[i].joint.joint_motor_signal
        if model == 0:
            NaoConnect.NaoSetAngles(MotorCommand)

            (FsrLeft, FsrRight, robPos, robOrient, HeadTouch, HandTouchLeft, HandTouchRight) = NaoConnect.NaoGetSensors()
            angles = NaoConnect.NaoGetAngles()
            Fsr = [FsrLeft, FsrRight]

            time2 = time.time()
            while (time2 - time1) < 0.05:
                time2 = time.time()
            # print 'time diff: ', time2 - time1
            time1 = time2
        else:
            frw_temp = frwKin(MotorCommand[2:7])
            XX = frw_temp[0]
            YY = frw_temp[1]
            theta_SR_ER = invKin(XX[2], YY[2])
            counter = 0
            for i in JointsList:
                angles[i] = MotorCommand[i]
                # angles[i] = theta_SR_ER[counter]
                counter = counter + 1
                # Update sensor neurons first before update the CPG
            # XX_mat.append(XX[2])
            # YY_mat.append(YY[2])
            XX_mat.append(XX)
            YY_mat.append(YY)

        # Update all commands and all sensors
        All_Command.append(MotorCommand[:])
        All_Joints_Sensor.append(angles)
        # print 'angles [3]: ', angles[3]*180.0/math.pi, ' angles [5]: ', angles[5]*180.0/math.pi


    if model == 0:
        ArmTrajX, ArmTrajY = naoplot.fPlot2DLeftArm(All_Command, All_Joints_Sensor, 20, 'Left Arm',
                                                    'arm_off')  # Drawing the arm :arm_off / arm_on
        np.savetxt('allcomand.out', np.array(All_Command))
        np.savetxt('alljsensor.out', np.array(All_Joints_Sensor))
        # np.savetxt('alliinj.out', np.array(All_Iinj))
    elif model == 1:
        ArmTrajX = XX_mat
        ArmTrajY = YY_mat
    return (ArmTrajX, ArmTrajY, All_Command)#, All_Iinj)#, All_Command, All_Joints_Sensor)

