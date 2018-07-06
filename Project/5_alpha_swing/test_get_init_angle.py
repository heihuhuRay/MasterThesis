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
from swing_on_real_Nao import *
from q_learning_data_structure import *




def main_walk():
    TextObj.say('Attention, Fall Manager is Disabled.')
    movObj.setFallManagerEnabled(False) # True False
    go_to_init_pos()
    test_init = NaoConnect.NaoGetAngles()
    print('  L_ANKLE_PITCH:', test_init[L_ANKLE_PITCH])
    print('  R_ANKLE_PITCH:', test_init[R_ANKLE_PITCH])
    print('   L_KNEE_PITCH:', test_init[L_KNEE_PITCH])
    print('   R_KNEE_PITCH:', test_init[R_KNEE_PITCH])
    print('    L_HIP_ROLL :', test_init[L_HIP_ROLL])
    print('    R_HIP_ROLL :', test_init[R_HIP_ROLL])
    print('    L_HIP_PITCH:', test_init[L_HIP_PITCH])
    print('    R_HIP_PITCH:', test_init[R_HIP_PITCH])
    print('L_HIP_YAW_PITCH:', test_init[L_HIP_YAW_PITCH])
    print('R_HIP_YAW_PITCH:', test_init[R_HIP_YAW_PITCH])
    # Disable Fall Manager
    release_arm_stiffness() 
    TextObj.say('Please hold my wrist.')
    time.sleep(4)
    swing_on_Nao([0.042, 0.011, 0.02, 0.022], 800)
    fPlotJointCommandSensor(All_Command,All_Sensor,L_HIP_ROLL,'L_HIP_ROLL')
    fPlotJointCommandSensor(All_Command,All_Sensor,L_ANKLE_ROLL,'L_ANKLE_ROLL')
    fPlotJointCommandSensor(All_Command,All_Sensor,L_HIP_PITCH,'L_HIP_PITCH')

if __name__ == "__main__":
    main_walk()