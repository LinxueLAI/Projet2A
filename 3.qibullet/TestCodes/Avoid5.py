#!/usr/bin/env python
# coding: utf-8
import cv2
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
import pybullet
import pybullet_data
import math

data_scan=[0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0]#45
def braitenberg(left_scan,front_scan,right_scan):
    # data_scan=[0,0,0,0,0,0,0,0,0]#9
    data_scan[0]=left_scan[0]
    data_scan[1]=left_scan[1]
    data_scan[2]=left_scan[2]
    data_scan[3]=left_scan[3]
    data_scan[4]=left_scan[4]
    data_scan[5]=left_scan[5]
    data_scan[6]=left_scan[6]
    data_scan[7]=left_scan[7]
    data_scan[8]=left_scan[8]
    data_scan[9]=left_scan[9]
    data_scan[10]=left_scan[10]
    data_scan[11]=left_scan[11]
    data_scan[12]=left_scan[12]
    data_scan[13]=left_scan[13]
    data_scan[14]=left_scan[14]

    for k in range(0,15):
        # print("k="+str(k))
        data_scan[15+k]=front_scan[k]

    for j in range(0,15):
        data_scan[30+j]=right_scan[j]

    for i in range(0,45):
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
    for i in range(0,45):
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        pass
    print("vLeft="+str(vLeft))
    print("vRight="+str(vRight))
    return vLeft,vRight

if __name__=='__main__':
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
    ##Obstacles##
    pybullet.loadURDF(
    "samurai.urdf",
    basePosition=[0, 0, -0.1],
    globalScaling=0.8,
    physicsClientId=client)
    pybullet.loadURDF(
    "sphere_small.urdf",
    basePosition=[2, 0, 0.0],
    globalScaling=10.0,
    physicsClientId=client)
    # pybullet.loadURDF(
    # "humanoid.urdf",
    # basePosition=[2, 2, 0.0],
    # globalScaling=1.0,
    # physicsClientId=client)
    # pybullet.loadURDF(
    # "lego.urdf",
    # basePosition=[-2, 0, 0.0],
    # globalScaling=10.0,
    # physicsClientId=client)

    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[-2, 0, 0.0],
    globalScaling=10.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[-4, 2, 0.0],
    globalScaling=10.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[-4, 4, 0.0],
    globalScaling=10.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[-4, 6, 0.0],
    globalScaling=10.0,
    physicsClientId=client)

    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[-4, -2, 0.0],
    globalScaling=10.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[-4, -4, 0.0],
    globalScaling=10.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[-4, -6, 0.0],
    globalScaling=10.0,
    physicsClientId=client)

    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[4, 2, 0.0],
    globalScaling=10.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[4, 4, 0.0],
    globalScaling=10.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[4, 6, 0.0],
    globalScaling=10.0,
    physicsClientId=client)

    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[4, -2, 0.0],
    globalScaling=10.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[4, -4, 0.0],
    globalScaling=10.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "lego.urdf",
    basePosition=[4, -6, 0.0],
    globalScaling=10.0,
    physicsClientId=client)

    # pybullet.loadURDF(
    # "pendulum5.urdf",
    # basePosition=[4, -8, 0.0],
    # globalScaling=4.0,
    # physicsClientId=client)
    # pybullet.loadURDF(
    # "pendulum5.urdf",
    # basePosition=[-4, -8, 0.0],
    # globalScaling=4.0,
    # physicsClientId=client)

    # pybullet.loadURDF(
    # "sphere2red_nocol.urdf",
    # basePosition=[-4, -8, 0.0],
    # globalScaling=5.0,
    # physicsClientId=client)
    ##Obstacles##
    
    pepper.showLaser(True)
    pepper.subscribeLaser()
    pepper.goToPosture("Stand", 0.6)
    time.sleep(1)
    
    while True:
        # pepper.moveTo(0.3,0.0,0.0,_async=True)
        noDetectionDist=1.5
        maxDetectionDist=0.5
        detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#45
        braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6,-1.8,-2.0,-2.2,-2.4,-2.6,-2.8,-3.0,
        -3.1,-3.2,-3.3,-3.4,-3.5,-3.6,-3.7,-3.8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
        0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]#1.2,-1.6,-2.2,-3.2,-4.2,

        braitenbergR=[-3.8,-3.7,-3.6,-3.5,-3.4,-3.3,-3.2,-3.1,-3.0,-2.8,-2.6,-2.4,-2.2,-2.0,-1.8,
        -1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
        0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        pepper.subscribeLaser()
        front_scan = pepper.getFrontLaserValue()
        right_scan = pepper.getRightLaserValue()
        left_scan = pepper.getLeftLaserValue()
        # print("left_scan="+str(left_scan))
        # print("front_scan="+str(front_scan))
        # print("right_scan="+str(right_scan))

        vLeft,vRight=braitenberg(left_scan,front_scan,right_scan)
        x=(vLeft+vRight)/2
        R=0.5# or R=5
        theta=(vRight-vLeft)/(2*R)
        y=x*math.tan(theta)
        print("x="+str(x),";y="+str(y)+";x+y="+str(x+y)+";theta="+str(theta))
        pepper.moveTo(x,y,theta,_async=True)


