from robot import *
# import config
from robot_detect_image import *
import argparse
import sys
import time
pepper = Pepper("192.168.2.169", 9559)
#pepper.autonomous_life_off()
#test searchface
while True:
	pepper.searchFace()
	if cv2.waitKey(1) & 0xFF == ord('q'):
		pepper.stand()
		break
