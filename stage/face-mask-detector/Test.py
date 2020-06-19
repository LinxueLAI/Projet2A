from robotDetector import *

pepper = Pepper("192.168.2.169", 9559)
#pepper.autonomous_life_off()

# detect the face the first time,then track the face
pepper.trackFace()
