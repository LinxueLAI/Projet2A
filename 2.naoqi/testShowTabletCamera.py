from robot import *
# import config
import argparse
import sys
import time
pepper = Pepper("192.168.2.169", 9559)
# pepper.clean_tablet()
# pepper.tablet_service.hideWebview()
pepper.tablet_show_web("http://www.google.fr")
# # pepper.tablet_service.enableWifi()
# # pepper.tablet_service.showWebview("http://www.google.fr")
# subprocess.Popen(["python2", "-m", "SimpleHTTPServer"])
# # pepper.share_localhost("/Users/llx33/Desktop/Projet2A/Projet2A/2.naoqi/")
# pepper.subscribe_camera("camera_top", 2, 30)
# while True:
#     pepper.show_tablet_camera("camera top")
#     # pepper.tablet_show_web("https://www.ciirc.cvut.cz")
#     pepper.tablet_show_web("http://127.0.0.1:8000/tmp/camera.png")
    # need to know remote ip: "http://" + remote_ip + ":8000/tmp/camera.png"
    # remote_ip = socket.gethostbyname(socket.gethostname())
    
