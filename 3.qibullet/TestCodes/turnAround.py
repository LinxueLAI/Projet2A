#!/usr/bin/env python
# coding: utf-8
import cv2
import time
from qibullet import SimulationManager
from qibullet import PepperVirtual
import pybullet
import pybullet_data
import os
import threading
import inspect
import ctypes

class Job1(threading.Thread): 
    def __init__(self, *args, **kwargs):
        super(Job1, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()     # 用于暂停线程的标识
        self.__flag.set()       # 设置为True
        self.__running = threading.Event()      # 用于停止线程的标识
        self.__running.set()      # 将running设置为True
        #self.turnAround()

    def run(self):
        #self.turnAround() 
        while self.__running.isSet():
            self.__flag.wait()      # 为True时立即返回, 为False时阻塞直到内部的标识位为True后返回
            print(time.time())
            time.sleep(1) 
    def pause(self):
        self.__flag.clear()     # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()    # 设置为True, 让线程停止阻塞
    def turnAround(self):
        i=4
        while i>0:
            pepper.moveTo(2.0,0.0,0.0)
            print("1:"+str(os.getpid()))
            time.sleep(1)
            print("2:"+str(os.getpid()))
            pepper.moveTo(0.0,0.0,1.5707)
            print("3:"+str(os.getpid()))
            #pepper.move(1.0, 0.5, 0.5)
            i=i-1
            time.sleep(1)
            print("4:"+str(os.getpid()))
    def detect(self):
        while True:
            pepper.subscribeLaser()
            front_scan = pepper.getFrontLaserValue()
            #right_scan=pepper.getRightLaserValue()
            if all(laser == 5.6 for laser in front_scan):
                print("Nothing detected")
            else:
                print("Detected:")
                print(front_scan)
                #i=0
                #for i in range(0,15):
                    #print("front_scan["+str(i)+"]:"+str(front_scan[i]))
                    
    def stop(self):
        self.__flag.set()       # 将线程从暂停状态恢复, 如何已经暂停的话
        self.__running.clear()        # 设置为False </pre>
class Job2(threading.Thread): 
    def __init__(self, *args, **kwargs):
        super(Job2, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()     # 用于暂停线程的标识
        self.__flag.set()       # 设置为True
        self.__running = threading.Event()      # 用于停止线程的标识
        self.__running.set()      # 将running设置为True
        self.detect()

    def run(self): 
        while self.__running.isSet():
            self.__flag.wait()      # 为True时立即返回, 为False时阻塞直到内部的标识位为True后返回
            print(time.time())
            time.sleep(1) 
    def pause(self):
        self.__flag.clear()     # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()    # 设置为True, 让线程停止阻塞
    def turnAround(self):
        i=4
        while i>0:
            pepper.moveTo(2.0,0.0,0.0)
            print("1:"+str(os.getpid()))
            time.sleep(1)
            print("2:"+str(os.getpid()))
            pepper.moveTo(0.0,0.0,1.5707)
            print("3:"+str(os.getpid()))
            #pepper.move(1.0, 0.5, 0.5)
            i=i-1
            time.sleep(1)
            print("4:"+str(os.getpid()))
    def detect(self):
        while True:
            pepper.subscribeLaser()
            front_scan = pepper.getFrontLaserValue()
            #right_scan=pepper.getRightLaserValue()
            if all(laser == 5.6 for laser in front_scan):
                print("Nothing detected")
            else:
                print("Detected:")
                print(front_scan)
                #i=0
                #for i in range(0,15):
                    #print("front_scan["+str(i)+"]:"+str(front_scan[i]))
                    
    def stop(self):
        self.__flag.set()       # 将线程从暂停状态恢复, 如何已经暂停的话
        self.__running.clear()        # 设置为False </pre>

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

def turnAround():
    i=4
    while i>0:
        pepper.moveTo(2.0,0.0,0.0)
        print("1:"+str(os.getpid()))
        time.sleep(1)
        print("2:"+str(os.getpid()))
        pepper.moveTo(0.0,0.0,1.5707)#90°
        print("3:"+str(os.getpid()))
        #pepper.move(1.0, 0.5, 0.5)
        i=i-1
        time.sleep(1)
        print("4:"+str(os.getpid()))

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
            #i=0
            #for i in range(0,15):
                #print("front_scan["+str(i)+"]:"+str(front_scan[i]))
            if min(front_scan)<0.5:
                t1.pause()
                pepper.move(0.0,0.0,0.5)
                print("t1 is pausing")

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

   # stop_thread(t2)
print ("退出线程")
