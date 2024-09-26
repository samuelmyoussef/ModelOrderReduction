# -*- coding: utf-8 -*-
import os
import platform
from sys import argv

#   STLIB IMPORT
try:
	from stlib3.scene.wrapper import Wrapper
except:
    raise ImportError("ModelOrderReduction plugin depend on SPLIB"\
                     +"Please install it : https://github.com/SofaDefrost/STLIB")

# MOR IMPORT
from mor.utility import sceneCreation
from mor.utility import utility as u 

slash = '/'
if "Windows" in platform.platform():
    slash = "\\"

# Our Original Scene IMPORT
originalScene = r'/home/samuelyoussef/Documents/SOFA/Plugins/ModelOrderReduction/contacts_reduction/Finger/fingerWithCollision_soft.py'
originalScene = os.path.normpath(originalScene)
originalScene = u.load_source(originalScene.split(slash)[-1], originalScene)

paramWrapper = ('/solverNode/finger', {'paramForcefield': {'prepareECSW': True, 'modesPath': '/home/samuelyoussef/Documents/SOFA/Plugins/ModelOrderReduction/contacts_reduction/Finger/reduced_soft/data/modes.txt', 'periodSaveGIE': 6, 'nbTrainingSet': 22}, 'paramMORMapping': {'input': '@../MechanicalObject', 'modesPath': '/home/samuelyoussef/Documents/SOFA/Plugins/ModelOrderReduction/contacts_reduction/Finger/reduced_soft/data/modes.txt'}})

def createScene(rootNode):

    if (len(argv) > 1):
        stateFileName = str(argv[1])
    else:	
        stateFileName="stateFile.state"
    originalScene.createScene(rootNode)

    path , param = paramWrapper
    pathToNode = path[1:]

    sceneCreation.createDebug(rootNode,pathToNode,stateFileName)