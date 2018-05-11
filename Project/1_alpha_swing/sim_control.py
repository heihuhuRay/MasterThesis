import vrep as v
import time

global clientID

clientID = 0

def start_sim():
    clientID = v.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1:
        v.simxStartSimulation(clientID, v.simx_opmode_oneshot_wait)

def stop_sim():
    clientID = v.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1:
        v.simxStopSimulation(clientID, v.simx_opmode_oneshot_wait)
