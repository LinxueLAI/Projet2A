#!/usr/bin/env python
# coding: utf-8
import cv2
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
import pybullet
import pybullet_data
import math
if __name__=='__main__':
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[5, 2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[5, 1, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[5, 0, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[5, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[5, 4, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[0, 5, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[1, 5, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[2, 5, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[3, 5, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[4, 5, 0.5],
    globalScaling=1.0,
    physicsClientId=client)

    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-5, 2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-5, 1, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-5, 0, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-5, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-5, 4, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[0, -5, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[1, -5, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[2, -5, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[3, -5, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[4, -5, 0.5],
    globalScaling=1.0,
    physicsClientId=client)

    pepper.showLaser(True)
    pepper.subscribeLaser()
    pepper.goToPosture("Stand", 0.6)
    time.sleep(1)
    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)#or ID_CAMERA_BOTTOM,ID_CAMERA_DEPTH

    time.sleep(1)
    #print(pepper.getPosition())
    noDetectionDist=0.7
    maxDetectionDist=0.2
    detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    braitenberg=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2]
    while True:
        pepper.moveTo(0.1,0.0,0.0,_async=True)
        pepper.subscribeLaser()
        front_scan = pepper.getFrontLaserValue()
        #right_scan=pepper.getRightLaserValue()
        if all(laser == 5.6 for laser in front_scan):
            print("Nothing detected")
        else:
            # print("Detected:")
            # print(front_scan)
            for i in range(0,15):
                dist=front_scan[i]
                if (dist<noDetectionDist):
                    if (dist<maxDetectionDist):
                        dist=maxDetectionDist
                        pass
                    detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
                else:
                    detect[i]=0
                    pass
                pass
            
            while (min(front_scan)<=0.7):
                v=2.0
                for k in range(0,15):
                    v=v+braitenberg[k]*detect[k]
                    #print("v="+str(v))
                pepper.goToPosture("Stand", 0.6)
                # pepper.moveTo(-0.2,0.0,0.0,_async=True)
                print("turn left"+str(((2-v)*math.pi)))
                theta=((2-v)*math.pi)/3
                if theta>math.pi:
                    theta=math.pi/3
                elif theta<0:
                    theta=0.15
                pepper.moveTo(0.0,0.0,theta*2,_async=True)
                time.sleep(1)
                front_scan = pepper.getFrontLaserValue()
