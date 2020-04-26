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
 
def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)
#stop thread


def turnAround():
    i=4
    while i>0:
        pepper.moveTo(2.0,0.0,0.0)
        time.sleep(1)
        pepper.moveTo(0.0,0.0,1.5707)
        #pepper.move(1.0, 0.5, 0.5)
        i=i-1
        time.sleep(1)

def detect():
    while True:
        pepper.subscribeLaser()
        front_scan = pepper.getFrontLaserValue()
        #right_scan=pepper.getRightLaserValue()
        if all(laser == 5.6 for laser in front_scan):
            print("Nothing detected")
        else:
            print("Detected:")
            print(front_scan)
            for i in range(0,15):
                #print("front_scan["+str(i)+"]:"+str(front_scan[i]))
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
            vLeft=2.0
            if min(front_scan)<=0.2:
                for k in range(0,15):
                    vLeft=vLeft+braitenberg[k]*detect[k]
                    print("vLeft="+str(vLeft))
                    #pepper.move(0.0,2.0-vLeft,0.5707)
                pepper.move(vLeft,0.0,(2-vLeft)/2)   
        time.sleep(0.01)


flag = 0
if __name__=='__main__':
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    pybullet.loadURDF(
    "cube_no_rotation.urdf",#duck_vhacd.urdf
    basePosition=[2, 2, 0.5],
    globalScaling=1.0,
    physicsClientId=client)

    pepper.showLaser(True)
    pepper.subscribeLaser()
    pepper.goToPosture("Stand", 0.6)
    time.sleep(1)
    pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP)#or ID_CAMERA_BOTTOM,ID_CAMERA_DEPTH
    time.sleep(1)

    threads = []
    noDetectionDist=0.5
    maxDetectionDist=0.2
    detect=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    braitenberg=[-0.2,-0.4,-0.6,-0.8,-1.0,-1.2,-1.4,-1.6,-1.4,-1.2,-1.0,-0.8,-0.6,-0.4,-0.2]
    v0=2

    t1 = threading.Thread(target=detect)
    threads.append(t1)

    n=0
    for t in threads:
        t.start()
        print(n)
        n=n+1

    for t in threads:
        t.join()

    pepper.unsubscribeLaser()
    stop_thread(t1)
    #stop_thread(t2)
print ("退出线程")
