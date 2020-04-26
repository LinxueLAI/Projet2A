#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use Tracking Module to Track a Face"""

import qi
import argparse
import sys
import time


def main(session, faceSize):
    """
    This example shows how to use ALTracker with face.
    """
    # Get the services ALTracker and ALMotion.

    motion_service = session.service("ALMotion")
    tracker_service = session.service("ALTracker")

    # First, wake up.
    motion_service.wakeUp()

    # Add target to track.
    targetName = "Face"
    print(faceSize)
    faceWidth = faceSize
    b = tracker_service.registerTarget(targetName, faceWidth)
    print(b)
    # Then, start tracker.
    a= tracker_service.track(targetName)
    print(a)

    print "ALTracker successfully started, now show your face to robot!"
    print "Use Ctrl+c to stop this script."

    try:
        while True:
            print("1")
            tracker_service.track(targetName)
            time.sleep(1)
            print("2")
    except KeyboardInterrupt:
        print
        print "Interrupted by user"
        print "Stopping..."

    # Stop tracker.
    tracker_service.stopTracker()
    tracker_service.unregisterAllTargets()
    # make pepper sleep
    #motion_service.rest()

    print "ALTracker stopped."


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.2.169",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    parser.add_argument("--facesize", type=float, default=0.1,
                        help="Face width.")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session, args.facesize)