#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
from naoqi import ALProxy
import argparse
import almath
import math
import qi
import naoqi
import sys
import time
import HumanGreeter
def main(session):
    #print("errorpos = "+session.positionErrorThresholdPos)
    #print("errorpAng = "+session.positionErrorThresholdAng)
    #self.positionErrorThresholdPos = 0.01
    #self.positionErrorThresholdAng = 0.03
    ip = "192.168.2.169"
    #imagine you have a NAOqi proxy on almemory
    mem = ALProxy("ALMemory", ip, 9559)
    #get a qi session
    motion = ALProxy("ALMotion", ip, 9559)
    #speak
    tts = ALProxy("ALTextToSpeech", "192.168.2.169", 9559)

    #Get the service ALTracker
    #tracker_service= ALProxy("ALTracker", ip, 9559)
    # Stop tracker.
    #tracker_service.stopTracker()
    #tracker_service.unregisterAllTargets()

    #print(setOrthogonalSecurityDistance())              
    posture = ALProxy("ALRobotPosture", ip, 9559)  
    motion.wakeUp()
    posture.goToPosture("StandInit", 0.5)
    motion.setStiffnesses("Body", 1.0)
    motion.moveInit()
    print("OrthogonalSecurityDistance=")
    print(motion.getOrthogonalSecurityDistance())
    print("TangentialSecurityDistance=")
    print(motion.getTangentialSecurityDistance())
    #minimum distance between the robot and any obstacle during a move
    motion.setOrthogonalSecurityDistance(0.1)#default:0.4
    motion.setTangentialSecurityDistance(0.05)#defaut:0.1
    i = 4
    tts.say("salut!Je peux marcher!")
    while i>0:#marcher dans une carée

        memory_service = session.service("ALMemory")
        sonar_service = session.service("ALSonar")

        # Subscribe to sonars, this will launch sonars (at hardware level)
        # and start data acquisition.
        sonar_service.subscribe("myApplication")

        # Now you can retrieve sonar data from ALMemory.
        # Get sonar left first echo (distance in meters to the first obstacle).
        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/X/Sensor/Value")
        print(4-i)
        print(left)

        # Same thing for right.
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/Y/Sensor/Value")
        print(4-i)
        print(right)

        # Unsubscribe from sonars, this will stop sonars (at hardware level)
        # sonar_service.unsubscribe("myApplication")
        id1 = motion.post.moveTo(2.0, 0.0, 0.0)#2 metre;  1 metre ; 0.5 = 90 degres
        i = i - 1
        motion.wait(id1, 0)
        id2 = motion.post.moveTo(0.0, 0.0, 0.5 * math.pi)
        tts.say("Je tourne a gauche!")
        motion.wait(id2, 0)

    #tts.say("OK,Je stop.")
    #motion.rest()
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.2.169",
                        help="Robot IP address. On robot or Local Naoqi: use '192.168.2.169'.")
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