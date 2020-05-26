from robot import *
# import config
import argparse
import sys
import time
pepper = Pepper("192.168.2.169", 9559)

pepper.exploration_mode(1.0)# is that necessary?
pepper.robot_localization()
pepper.navigate_to(2.0, 0.0)
"""
        Navigate robot in map based on exploration mode
        or load previously mapped enviroment.

        .. note:: Before navigation you have to run localization of the robot.

        .. warning:: Navigation to 2D point work only up to 3 meters from robot.

        :Example:

        >>> pepper.robot_localization()
        >>> pepper.navigate_to(1.0, 0.3)

        :param x: X axis in meters
        :type x: float
        :param y: Y axis in meters
        :type y: float
"""