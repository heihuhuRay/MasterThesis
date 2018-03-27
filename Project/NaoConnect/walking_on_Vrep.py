import sys
import NaoConnect
import time
import numpy as np
import vrep

from SetTiming import *
from MLMPCPG import *
from NAOMotor import *
from random import randint

######################################

if NaoConnect.NaoRobotConnect.RealNaoRobot:
    print "RealNaoRobot: ", NaoConnect.NaoRobotConnect.RealNaoRobot[0]

if NaoConnect.NaoVrepConnect.NaoVrep:
    print "NaoVrep: ", NaoConnect.NaoVrepConnect.NaoVrep[0]

NAOosON = NaoConnect.NaoRobotConnect.RealNaoRobot or NaoConnect.NaoVrepConnect.NaoVrep

if NAOosON == []:
    sys.exit("No robot or simulation connected..!")

print "NAOosON : ", NAOosON

#######################################################