from robot import *
# import config
import argparse
import sys
import time
pepper = Pepper("192.168.2.169", 9559)
pepper.motion_service.wakeUp()
pepper.pick_a_volunteer()