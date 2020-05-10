#!/usr/bin/env python
# coding: utf-8
import cv2
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
import pybullet
import pybullet_data
import math
def braitenberg(left_scan,front_scan,right_scan):
    data_scan=[0,0,0,0,0,0,0,0,0]#9
    # data_scan[0]=left_scan[0]
    # data_scan[1]=left_scan[7]
    # data_scan[2]=left_scan[14]
    # data_scan[3]=front_scan[0]
    # data_scan[4]=front_scan[7]
    # data_scan[5]=front_scan[14]
    # data_scan[6]=right_scan[0]
    # data_scan[7]=right_scan[7]
    # data_scan[8]=right_scan[14]
    data_scan[0]=front_scan[0]
    data_scan[1]=front_scan[2]
    data_scan[2]=front_scan[4]
    data_scan[3]=front_scan[6]
    data_scan[4]=front_scan[7]
    data_scan[5]=front_scan[8]
    data_scan[6]=front_scan[10]
    data_scan[7]=front_scan[12]
    data_scan[8]=front_scan[14]
    print("data_scan="+str(data_scan))
    for i in range(0,9):
        dist=data_scan[i]
        if (dist<noDetectionDist):
            if (dist<maxDetectionDist):
                dist=maxDetectionDist
                pass
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else:
            detect[i]=0
            pass
        pass                
    vLeft=2.0
    vRight=2.0
    for i in range(0,9):
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        pass
    print("vLeft=")
    print(vLeft)
    print("vRight=")
    print(vRight)
    return vLeft,vRight

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
    
    while True:
        pepper.moveTo(2.0,0.0,0.0,_async=True)
        noDetectionDist=0.5
        maxDetectionDist=0.2
        detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        braitenbergL=[-0.2,-0.4,-0.8,-1.0,-1.2,0.0,0.0,0.0,0.0]
        braitenbergR=[-1.2,-1.0,-0.8,-0.4,-0.2,0.0,0.0,0.0,0.0]

        pepper.subscribeLaser()
        front_scan = pepper.getFrontLaserValue()
        right_scan = pepper.getRightLaserValue()
        left_scan = pepper.getLeftLaserValue()

        x,y=braitenberg(left_scan,front_scan,right_scan)
        # if(x<=0):
        #     theta=1.57
        # elif y<0:
        #     theta=-1.57
        # else:
        #     theta=max(y/x,0)
        theta=max(y/x,0)
        print("x="+str(x),";y="+str(y)+";x+y="+str(x+y)+";theta="+str(theta))
        # pepper.moveTo(x,0.0,0.0,_async=True)
        # pepper.moveTo(0.0,y,0.0,_async=True)
        pepper.moveTo(x,0.0,0.0,_async=True)
        pepper.moveTo(0.0,y,0.0,_async=True)

        # time.sleep(1)
        pepper.moveTo(0.0,0.0,theta,_async=True)
        time.sleep(1)

