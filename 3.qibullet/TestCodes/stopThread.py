# Code to execute in an independent thread 
import time 
import os
def countdown(n): 
    while n > 0: 
        print('T-minus', n) 
        n -= 1
        print(os.getpid())
        time.sleep(1)
    if t.is_alive():
        print("alive")
    else:
        print("stopping")
   
          
# Create and launch a thread 
from threading import Thread 
t = Thread(target = countdown, args =(10, ))

t.start()
if t.is_alive(): 
    print('Still running') 
else: 
    print('Completed')
t.join()
if t.is_alive(): 
    print('Still running') 
else: 
    print('Completed')  