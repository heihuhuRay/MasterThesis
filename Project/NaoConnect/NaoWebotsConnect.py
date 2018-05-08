# -*- coding: utf-8 -*-
"""
This module contains the inisialization code to connect to Webots simulator. It also has functions to read and set nao robot joints and also read sonsors in Webots simulation. 
It is a middlewear between the simulator and the user code. 


Created on Mon May 29 00:36:46 2017

@todo: all

@author: nasjo
"""

global NaoWebots
NaoQi = []
NaoWebots = []

NAOIP = []
PORT = []

#### NAOQI 
try: 
    import naoqi
    from naoqi import ALProxy
    from naoqi import ALModule
    NaoQi.append(1)
    print ('Naoqi is imported.')            
except:
    print ('-----------------------------')
    print ('Error in importing naoqi!')
    print ('Make sure naoqi is installed')
    print ('-----------------------------')
    print ('')

if NaoQi:
    try: 
        NAOIP = "127.0.0.1"
        PORT= 9559
        movObj = ALProxy("ALMotion",NAOIP,PORT)
        postObj = ALProxy("ALRobotPosture",NAOIP,PORT)
        memProxy = ALProxy("ALMemory",NAOIP,PORT)
        NaoWebots.append(1)
    except: 
        print ('Cannot connect to Webots!')



def NaoSetAnglesWebots(angles):
    print "NAOIP", NAOIP
    return
    
