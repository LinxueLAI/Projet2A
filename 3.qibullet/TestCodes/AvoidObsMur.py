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
    ##Obstacles##
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[3, 2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[3, 1, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[3, 0, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[3, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[3, -1, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[3, -2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[3, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[2, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[1, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[0, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)

    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-1, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-2, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-3, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)

    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-3, -2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-3, -1, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-3, 0, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-3, 1, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-3, 2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-3, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-2, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[-1, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[0, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[1, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",
    basePosition=[2, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    ##Obstacles##
    pepper.showLaser(True)
    pepper.subscribeLaser()
    pepper.goToPosture("Stand", 0.6)
    time.sleep(1)
    # handle=pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)#or ID_CAMERA_BOTTOM,ID_CAMERA_DEPTH
    # pepper.subscribeCamera(PepperVirtual.ID_CAMERA_BOTTOM)

    # while True:
    #     img = pepper.getCameraFrame()
    #     cv2.imshow("bottom camera", img)
    #     cv2.waitKey(1)
    # time.sleep(1)

    #print(pepper.getPosition())
    noDetectionDist=0.5
    maxDetectionDist=0.2
    detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    braitenbergR=[-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    while True:
        pepper.moveTo(0.15,0.0,0.0,_async=True)
        # Move to the specified moveTo[x, y, theta] coordinates in the robot frame, 
        # synchronous call:
        #pepper.moveTo(1.0,1.0,1.0,frame=PepperVirtual.FRAME_ROBOT)## _async=False par default
        # asynchronous call:
        #pepper.moveTo(0.0,-1.0,1.39,frame=PepperVirtual.FRAME_WORLD,_async=True)

        # Apply a speed [x_vel, y_vel, theta_val] on the robot's base, in the 
        # robot frame. It is an asynchronous call
        #pepper.move(1.0, 0.5, 0.5)#The parameters are expressed in m/s for the translational speeds, and in rad/s for the rotational speed.
        
        pepper.subscribeLaser()
        front_scan = pepper.getFrontLaserValue()
        right_scan = pepper.getRightLaserValue()
        left_scan = pepper.getLeftLaserValue()
        #right_scan=pepper.getRightLaserValue()
        if all(laser == 5.6 for laser in front_scan):
            print("Nothing detected")
        else:
            while (min(front_scan)<=0.6):
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
                front_scan = pepper.getFrontLaserValue()
                right_scan = pepper.getRightLaserValue()
                left_scan = pepper.getLeftLaserValue()
                vL=4.0
                vR=4.0
                for k in range(0,15):
                    vL=vL+braitenbergL[k]*detect[k]
                    vR=vR+braitenbergR[k]*detect[k]

                print("vL="+str(vL))
                print("vR="+str(vR))
                
                theta=((4-max(vL,vR))*math.pi)/6
                print("turn: "+str(theta))
                
                if min(front_scan)<=0.5:
                    pepper.moveTo(-0.05,0.0,0.0,_async=True)

                if min(left_scan)<=0.5:
                    pepper.moveTo(0.0,-0.1,0.0,_async=True)
                    pepper.moveTo(0.0,0.0,-math.pi/6,_async=True)

                if min(right_scan)<=0.5:
                    pepper.moveTo(0.0,0.1,0.0,_async=True)
                    pepper.moveTo(0.0,0.0,math.pi/6,_async=True)
                pepper.moveTo(0.0,0.0,max(theta,0.15),_async=True)
                
