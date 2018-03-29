
import naoqi
from naoqi import ALProxy
import math
import time
import json
######################################
from SetTiming import *
from MLMPCPG import *
from NAOMotor import *
from random import randint
from alpha_value import *




file_path = "mylib\\payam\\"

model = 'robot'
# model = 'robot'
'''
if model == 'LArm2D':
    model = 1
    from inverseKin import invKin
    from forwardKin import frwKin

    print 'using forward kinematics'

    initPos = np.genfromtxt(file_path + "CurPos.txt", delimiter=",")
    initPos = np.array(initPos)
    
    model = 0
    import NaoConnect

    if NaoConnect.NaoRobotConnect.RealNaoRobot:
        print "RealNaoRobot: ", NaoConnect.NaoRobotConnect.RealNaoRobot[0]

    if NaoConnect.NaoVrepConnect.NaoVrep:
        print "NaoVrep: ", NaoConnect.NaoVrepConnect.NaoVrep[0]

    if NaoConnect.NaoWebotsConnect.NaoWebots:
        print "NaoWebots: ", NaoConnect.NaoQiConnect.NaoWebots[0]

    NAOosON = NaoConnect.NaoRobotConnect.RealNaoRobot or NaoConnect.NaoVrepConnect.NaoVrep or NaoConnect.NaoWebotsConnect.NaoWebots

    print "NAOosON : ", NAOosON
    if NAOosON == []:
        sys.exit("No robot or simulation connected..!")

    if NaoConnect.NaoRobotConnect.RealNaoRobot:
        NaoConnect.NaoRobotConnect.postObj.goToPosture("Stand", 0.8)

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
    time.sleep(2)

    print initPos[L_SHOULDER_PITCH:L_WRIST_YAW + 1]

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
        time.sleep(0.015)


    initPos[L_ANKLE_ROLL] = 0 * math.pi / 180.0
    initPos[R_ANKLE_ROLL] = 0 * math.pi / 180.0
    NaoConnect.NaoSetAngles(initPos)
'''

number_cpg = 26

global All_Command
global All_Sensor
global All_FSR, All_cur_out,All_RG_out
global All_PF_out, All_zmp, All_alpha

NAOIP = "0.0.0.0"
PORT= 9559

# Connect to the module ALMemoryProxy
memProxy = ALProxy("ALMemory", NAOIP, PORT)
data = memProxy.getData("WristForceSensor")


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


movObj = ALProxy("ALMotion",NAOIP,PORT)

#initPos = NaoConnect.NaoGetAngles()
initPos = movObj.getAngles('Body',True)
for i in range(0, len(myCont)):
    myCont[i].fUpdateInitPos(initPos[i])


for i in range(0, len(myCont)):
    myCont[i].fUpdateLocomotionNetwork(myT,initPos[i])
print 'Robot is ready to move..!!'
time.sleep(1)

all_joint_tm = 0.15

sigma_s_test = 1
sigma_f_test = 2.5

#Oscillatory pattern
RG_KneePitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_HipPitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_AnklePitch = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_HipRoll = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)
RG_AnkleRoll = RG_Patterns(sigma_f_test,sigma_s_test,1,all_joint_tm)

PF_AnkleRoll = PF_Patterns(alpha_AnkelRoll, 0)
PF_HipRoll = PF_Patterns(alpha_HipRoll, 0)
PF_HipPitch = PF_Patterns(alpha_HipPitch, 0)
PF_AnklePitch = PF_Patterns(alpha_AnkelPitch, 0)
PF_KneePitch = PF_Patterns(alpha_kneePitch, 0)

myCont[R_ANKLE_ROLL].fSetPatternRG(RG_AnkleRoll)
myCont[R_ANKLE_ROLL].fSetPatternPF(PF_AnkleRoll)

myCont[L_ANKLE_ROLL].fSetPatternRG(RG_AnkleRoll)
myCont[L_ANKLE_ROLL].fSetPatternPF(PF_AnkleRoll)

myCont[L_HIP_ROLL].fSetPatternRG(RG_HipRoll)
myCont[L_HIP_ROLL].fSetPatternPF(PF_HipRoll)

myCont[R_HIP_ROLL].fSetPatternRG(RG_HipRoll)
myCont[R_HIP_ROLL].fSetPatternPF(PF_HipRoll)

myCont[R_ANKLE_PITCH].fSetPatternRG(RG_AnklePitch)
myCont[R_ANKLE_PITCH].fSetPatternPF(PF_AnklePitch)

myCont[L_ANKLE_PITCH].fSetPatternRG(RG_AnklePitch)
myCont[L_ANKLE_PITCH].fSetPatternPF(PF_AnklePitch)

myCont[L_HIP_PITCH].fSetPatternRG(RG_HipPitch)
myCont[L_HIP_PITCH].fSetPatternPF(PF_HipPitch)

myCont[R_HIP_PITCH].fSetPatternRG(RG_HipPitch)
myCont[R_HIP_PITCH].fSetPatternPF(PF_HipPitch)

myCont[L_KNEE_PITCH].fSetPatternRG(RG_KneePitch)
myCont[L_KNEE_PITCH].fSetPatternPF(PF_KneePitch)

myCont[R_KNEE_PITCH].fSetPatternRG(RG_KneePitch)
myCont[R_KNEE_PITCH].fSetPatternPF(PF_KneePitch)


ExtInjCurr = 0
ExtInjCurr1 = 0

ExtInjCurr2 = 0
ExtInjCurr3 = 0

# initPos = NaoConnect.NaoGetAngles()
initPos = movObj.getAngles('Body',True)
for i in range(0, len(myCont)):
    myCont[i].fUpdateInitPos(initPos[i])
    myCont[i].joint.joint_motor_signal = myCont[i].joint.init_motor_pos
print initPos

# store the sensor data, used to store in json file
#   [[R_1, R_2, R_3, R_4, R_5, R_6, R_7], [L_1, L_2, L_3, L_4, L_5, L_6, L_7]]
sensor_data = {}
# !!! main loop
TextObj = ALProxy("ALTextToSpeech",NAOIP,PORT)
TextObj.say('Ready')

for I in range(0,500000):
    index = I % 500
    if index == 0:
        sensor_data[index] = sensor_data
    
    startTime = time.time()
    t= I*myT.T

    # inject positive current
    if I == 200:
        myT.T7 = t
        myT.T8 = myT.T7 + myT.signal_pulse_width
        tune_Ss_time_step = I +500

    # if t >= myT.T7 and t <= myT.T8:
    if t >= myT.T7 and t <= myT.T8:
        ExtInjCurr = 1
        ExtInjCurr1 = -1
        print "At ",I," current is injected"
    else:
        ExtInjCurr = 0
        ExtInjCurr1 = 0


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

    for i in [R_ANKLE_ROLL, R_HIP_ROLL,L_HIP_ROLL, L_ANKLE_ROLL]:
        myCont[i].fUpdateLocomotionNetwork(myT, initPos[i])



    for i in range(0, len(myCont)):
        MotorCommand[i]=myCont[i].joint.joint_motor_signal


    #NaoConnect.NaoSetAngles(MotorCommand)
    fractionMaxSpeed = 1.0
    movObj.setAngles('Body', MotorCommand , fractionMaxSpeed)
    #initPos = NaoConnect.NaoGetAngles()
    initPos = movObj.getAngles('Body',True)


# Writing JSON data
with open('data.json', 'w') as f:
    json.dump(sensor_data, f)

# TODO needs to test the code on Nao,then collect the data