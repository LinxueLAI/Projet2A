#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use showImage Method"""

import qi
import argparse
import sys
import time
import subprocess

def main(session):
    """
    This example uses the showImage method.
    To Test ALTabletService, you need to run the script ON the robot.
    """
    # Get the service ALTabletService.

    try:
        tabletService = session.service("ALTabletService")
        subprocess.Popen(["python2", "-m", "SimpleHTTPServer"])
        # Display a local image located in img folder in the root of the web server
        # The ip of the robot from the tablet is 198.18.0.1
        tabletService.showImage("http://127.0.0.1:8000/tmp/camera.png")

        time.sleep(3)

        # Hide the web view
        tabletService.hideImage()
    except Exception, e:
        print "Error was: ", e


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.2.169",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session)