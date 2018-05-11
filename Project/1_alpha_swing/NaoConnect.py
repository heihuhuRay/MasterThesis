# -*- coding: utf-8 -*-
"""
This module imports libraries to connect to the real robot ans simulation and connect to your robot (real robot if connected, nao in webots, nao in vrep). The avilable robot option will be controlled. If you want only to connect to the real robot, make sure that START bottom in your simulation is not pressed.

Created on Sun May 28 20:45:33 2017
@author: nasjo

@see: I{NaoVrepConnect} module
@see: I{NaoRobotConnect} module
@see: I{NaoWebotsConnect} module

"""





"""

 
"""
import NaoVrepConnect
import NaoWebotsConnect
import NaoRobotConnect 


def NaoSetAngles(ang):
    """
    This function set NAO angles.
    
    @param ang: input is an array of 26 angles[HEAD_YAW,HEAD_PITCH,L_SHOULDER_PITCH,L_SHOULDER_ROLL,L_ELBOW_YAW,L_ELBOW_ROLL,
    L_WRIST_YAW,L_HAND,L_HIP_YAW_PITCH,L_HIP_ROLL,L_HIP_PITCH,L_KNEE_PITCH,L_ANKLE_PITCH,
    L_ANKLE_ROLL,R_HIP_YAW_PITCH,R_HIP_ROLL,R_HIP_PITCH,R_KNEE_PITCH,R_ANKLE_PITCH,
    R_ANKLE_ROLL,R_SHOULDER_PITCH,R_SHOULDER_ROLL,R_ELBOW_YAW,R_ELBOW_ROLL,
    R_WRIST_YAW,R_HAND]
    """
    
    if NaoRobotConnect.RealNaoRobot:
        NaoRobotConnect.NaoSetAnglesRobot(ang)

    if NaoVrepConnect.NaoVrep:
        NaoVrepConnect.NaoSetAnglesVrep(ang)
        
    if NaoWebotsConnect.NaoWebots:
        NaoWebotsConnect.NaoSetAnglesWebots(ang)
            
    return


def NaoGetAngles():
    """
    This function get NAO angles.
    
    
    @return: An array of 26 angles:
    
    HEAD_YAW,HEAD_PITCH,L_SHOULDER_PITCH,L_SHOULDER_ROLL,L_ELBOW_YAW,L_ELBOW_ROLL,
    L_WRIST_YAW,L_HAND,L_HIP_YAW_PITCH,L_HIP_ROLL,L_HIP_PITCH,L_KNEE_PITCH,L_ANKLE_PITCH,
    L_ANKLE_ROLL,R_HIP_YAW_PITCH,R_HIP_ROLL,R_HIP_PITCH,R_KNEE_PITCH,R_ANKLE_PITCH,
    R_ANKLE_ROLL,R_SHOULDER_PITCH,R_SHOULDER_ROLL,R_ELBOW_YAW,R_ELBOW_ROLL,
    R_WRIST_YAW,R_HAND
    
    """
    if NaoRobotConnect.RealNaoRobot:
        return NaoRobotConnect.NaoGetAnglesRobot()

    if NaoVrepConnect.NaoVrep:
        return NaoVrepConnect.NaoGetAnglesVrep()
        
    if NaoWebotsConnect.NaoWebots:
        return NaoWebotsConnect.NaoGetAnglesWebots()

    return
     
        
def NaoGetSensors():
    """
    This function get NAO sensor date of predefined sensors.
    
    @return: A structure that contains: Fsr (8 values), Robot orientation (3 values), Robot position (3 values).
    
    """

    if NaoRobotConnect.RealNaoRobot:
        return NaoRobotConnect.NaoGetSensorsRobot()

    if NaoVrepConnect.NaoVrep:
        return NaoVrepConnect.NaoGetSensorsVrep()
        
    if NaoWebotsConnect.NaoWebots:
        return NaoWebotsConnect.NaoGetSensorsWebots()


    


        
def NaoGetVision():
    """
    This function get NAO vision sensor date.
    
    TO DO: all
    
    """

    if NaoRobotConnect.RealNaoRobot:
        return NaoRobotConnect.NaoGetVisionRobot()

    if NaoVrepConnect.NaoVrep:
        return NaoVrepConnect.NaoGetVisionVrep()
        
    if NaoWebotsConnect.NaoWebots:
        return NaoWebotsConnect.NaoGetVisionsWebots()


def NaoStop():
    """
    This function return your robot to a predefined posture and release the stiffness in case of real robot.
    
    """

    if NaoRobotConnect.RealNaoRobot:
        return NaoRobotConnect.NaoStopRobot()

    if NaoVrepConnect.NaoVrep:
        return NaoVrepConnect.NaoStopVrep()
        
    if NaoWebotsConnect.NaoWebots:
        return NaoWebotsConnect.NaoStopWebots()

