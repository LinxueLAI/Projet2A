from robot import *
# import config
import argparse
import sys
import time
pepper = Pepper("192.168.2.169", 9559)

## Test dance  ## marche bien 
pepper.dance(pepper.motion_service) 
