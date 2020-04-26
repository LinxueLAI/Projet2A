#!/usr/bin/env python # coding: utf-8
import threading 
import time
class Job(threading.Thread): 
    def __init__(self, *args, **kwargs):
        super(Job, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()     # 用于暂停线程的标识
        self.__flag.set()       # 设置为True
        self.__running = threading.Event()      # 用于停止线程的标识
        self.__running.set()      # 将running设置为True

    def run(self): 
        while self.__running.isSet():
            self.__flag.wait()      # 为True时立即返回, 为False时阻塞直到内部的标识位为True后返回
            print(time.time())
            time.sleep(1) 
    def pause(self):
        self.__flag.clear()     # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()    # 设置为True, 让线程停止阻塞

    def stop(self):
        self.__flag.set()       # 将线程从暂停状态恢复, 如何已经暂停的话
        self.__running.clear()        # 设置为False </pre>

a = Job()
a.start()
print(1)
time.sleep(3)
print(2)
a.pause()
print(3)
time.sleep(3)
print(4)
a.resume()
print(5)
time.sleep(3)
print(6)
a.pause()
print(7)
time.sleep(2)
print(8)
a.stop()
print(9)
