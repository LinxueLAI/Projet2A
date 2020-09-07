from robot import *
import sys
pepper = Pepper("192.168.2.169", 9559)
#pepper.autonomous_life_on()
# detect the face the first time,then track the face
# pepper.trackFace()
pepper.track_object('Face','None',0.1)
