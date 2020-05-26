from robot import *
# import config
import argparse
import sys
import time
pepper = Pepper("192.168.2.169", 9559)
pepper.share_localhost("/Users/michael/Desktop/Pepper/tmp/")
pepper.subscribe_camera("camera_top", 2, 30)
while True:
    pepper.show_tablet_camera("camera top")
    pepper.tablet_show_web("http://10.37.2.241:8000/tmp/camera.png")
    #need to know remote ip: "http://" + remote_ip + ":8000/tmp/camera.png"
    #remote_ip = socket.gethostbyname(socket.gethostname())
    
