#!/usr/bin/env python
# coding: utf-8
import cv2
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
import pybullet
import pybullet_data
import threading
import inspect
import ctypes
import math
def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")
#stop thread
def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)

if __name__=='__main__':
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[3, 2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[3, 1, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[3, 0, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[3, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[3, -1, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[3, -2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[3, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[2, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[1, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[0, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)

    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-1, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-2, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-3, -3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)

    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-3, -2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-3, -1, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-3, 0, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-3, 1, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-3, 2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-3, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-2, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[-1, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[0, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[1, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)
    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[2, 3, 0.5],
    globalScaling=1.0,
    physicsClientId=client)

    pepper.showLaser(True)
    pepper.subscribeLaser()
    pepper.goToPosture("Stand", 0.6)
    time.sleep(1)
    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)#or ID_CAMERA_BOTTOM,ID_CAMERA_DEPTH

    time.sleep(1)
    #print(pepper.getPosition())
    noDetectionDist=0.5
    maxDetectionDist=0.2
    detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    braitenbergR=[-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    while True:
        pepper.moveTo(0.15,0.0,0.0,_async=True)
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
                    #pepper.move(0.0,2.0-vLeft,0.5707)
                    # time.sleep(1)
                # if vL<vR:
                #     theta=((4-vL)*math.pi)/6
                # elif vL>vR:
                #     theta=-((4-vR)*math.pi)/6
                # else:
                #     break
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
                

print ("退出线程")
