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
from FaceDetected import HumanGreeter
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
    i = 1
    # tts.say("salut!Je peux marcher!")
    while i>0:#marcher dans une carée

        memory_service = session.service("ALMemory")
        sonar_service = session.service("ALSonar")

        # Subscribe to sonars, this will launch sonars (at hardware level)
        # and start data acquisition.
        sonar_service.subscribe("myApplication")

        # Now you can retrieve sonar data from ALMemory.
        # Get sonar left first echo (distance in meters to the first obstacle).
        # print(memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front"))
        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/X/Sensor/Value")
        print("X01="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/Y/Sensor/Value")
        print("Y01="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/X/Sensor/Value")
        print("X01="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg02/Y/Sensor/Value")
        print("Y02="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/X/Sensor/Value")
        print("X03="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/Y/Sensor/Value")
        print("Y03="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/X/Sensor/Value")
        print("X04="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg04/Y/Sensor/Value")
        print("Y04="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/X/Sensor/Value")
        print("X05="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/Y/Sensor/Value")
        print("Y05="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/X/Sensor/Value")
        print("X06="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg06/Y/Sensor/Value")
        print("Y06="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/X/Sensor/Value")
        print("X07="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/Y/Sensor/Value")
        print("Y07="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/X/Sensor/Value")
        print("X08="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg08/Y/Sensor/Value")
        print("Y08="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/X/Sensor/Value")
        print("X09="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/Y/Sensor/Value")
        print("Y09="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/X/Sensor/Value")
        print("X10="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg10/Y/Sensor/Value")
        print("Y10="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/X/Sensor/Value")
        print("X11="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/Y/Sensor/Value")
        print("Y11="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/X/Sensor/Value")
        print("X12="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg12/Y/Sensor/Value")
        print("Y12="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/X/Sensor/Value")
        print("X13="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/Y/Sensor/Value")
        print("Y13="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/X/Sensor/Value")
        print("X14="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/Y/Sensor/Value")
        print("Y14="+str(right))
        print(math.sqrt(math.pow(left,right)))

        left = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/X/Sensor/Value")
        print("X15="+str(left))
        right = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/Y/Sensor/Value")
        print("Y15="+str(right))
        print(math.sqrt(math.pow(left,right)))

        # Unsubscribe from sonars, this will stop sonars (at hardware level)
        # sonar_service.unsubscribe("myApplication")
        id1 = motion.post.moveTo(0.1, 0.0, 0.0)#2 metre;  1 metre ; 0.5 = 90 degres
        i = i - 1
        motion.wait(id1, 0)
        # id2 = motion.post.moveTo(0.0, 0.0, 0.5 * math.pi)
        # tts.say("Je tourne a gauche!")
        # motion.wait(id2, 0)

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