#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
"""Avoid obstacles by using algorithm braitenberg"""
from naoqi import ALProxy
import argparse
import almath
import math
import qi
import naoqi
import sys
import time
from FaceDetected import HumanGreeter
data_scan=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#27
noDetectionDist=0.8
maxDetectionDist=0.3
detect=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
braitenbergL=[-1.2,-1.4,-1.6,-1.8,-2.0,-2.2,-2.4,-2.6,-2.8,-3.0,-3.2,-3.4,-3.6,-4.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]#1.2,-1.6,-2.2,-3.2,-4.2,
braitenbergR=[-4.0,-3.6,-3.4,-3.2,-3.0,-2.8,-2.6,-2.4,-2.2,-2.0,-1.8,-1.6,-1.4,-1.2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
def braitenberg(left_scan,front_scan,right_scan):
    # data_scan=[0,0,0,0,0,0,0,0,0]#9
    data_scan[0]=left_scan[0]
    data_scan[1]=left_scan[1]
    data_scan[2]=left_scan[2]
    data_scan[3]=left_scan[3]
    data_scan[4]=left_scan[4]
    data_scan[5]=left_scan[5]
    data_scan[6]=left_scan[6]
    data_scan[7]=left_scan[7]
    data_scan[8]=left_scan[8]

    data_scan[9]=front_scan[0]
    data_scan[10]=front_scan[1]
    data_scan[11]=front_scan[2]
    data_scan[12]=front_scan[3]
    data_scan[13]=front_scan[4]
    data_scan[14]=front_scan[5]
    data_scan[15]=front_scan[6]
    data_scan[16]=front_scan[7]
    data_scan[17]=front_scan[8]

    data_scan[18]=right_scan[0]
    data_scan[19]=right_scan[1]
    data_scan[20]=right_scan[2]
    data_scan[21]=right_scan[3]
    data_scan[22]=right_scan[4]
    data_scan[23]=right_scan[5]
    data_scan[24]=right_scan[6]
    data_scan[25]=right_scan[7]
    data_scan[26]=right_scan[8]
    for i in range(0,27):
        dist=data_scan[i]
        if (dist<noDetectionDist):
            if (dist<maxDetectionDist):
                dist=maxDetectionDist
                pass
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else:
            detect[i]=0
            pass
        pass                
    vLeft=2.0
    vRight=2.0
    for i in range(0,27):
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
        pass
    print("vLeft="+str(vLeft))
    print("vRight="+str(vRight))
    return vLeft,vRight

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
    #posture standinit gives a more natural position for the robot. instead of being looking to the sky it looks in the direction of the horizon
    posture.goToPosture("StandInit", 0.5)
    
    #Since in this script we making a movement with the actuators, it is good practice to set the stiffness to the maximum in order for the robot to keep the final position of the actuators in the end of the movement.        
    motion.setStiffnesses("Body", 1.0)
    motion.moveInit()
    print("OrthogonalSecurityDistance=")
    print(motion.getOrthogonalSecurityDistance())
    print("TangentialSecurityDistance=")
    print(motion.getTangentialSecurityDistance())
    #minimum distance between the robot and any obstacle during a move
    motion.setOrthogonalSecurityDistance(0.3)#default:0.4
    motion.setTangentialSecurityDistance(0.1)#defaut:0.1
    # i = 1
    front_scan=[0,0,0,0,0,0,0,0,0]
    left_scan=[0,0,0,0,0,0,0,0,0]
    right_scan=[0,0,0,0,0,0,0,0,0]
    # tts.say("salut!Je peux marcher!")
    while True:
        # pepper.moveTo(0.3,0.0,0.0,_async=True)
        memory_service = session.service("ALMemory")
        sonar_service = session.service("ALSonar")
        sonar_service.subscribe("myApplication")

        front_scan[0] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg01/X/Sensor/Value")
        front_scan[1] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg03/X/Sensor/Value")
        front_scan[2] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg05/X/Sensor/Value")
        front_scan[3] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg07/X/Sensor/Value")
        front_scan[4] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg09/X/Sensor/Value")
        front_scan[5] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg11/X/Sensor/Value")
        front_scan[6] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg13/X/Sensor/Value")
        front_scan[7] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg14/X/Sensor/Value")
        front_scan[8] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Front/Horizontal/Seg15/X/Sensor/Value")

        left_scan[0] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg01/X/Sensor/Value")
        left_scan[1] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg03/X/Sensor/Value")
        left_scan[2] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg05/X/Sensor/Value")
        left_scan[3] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg07/X/Sensor/Value")
        left_scan[4] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg09/X/Sensor/Value")
        left_scan[5] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg11/X/Sensor/Value")
        left_scan[6] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg13/X/Sensor/Value")
        left_scan[7] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg14/X/Sensor/Value")
        left_scan[8] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Left/Horizontal/Seg15/X/Sensor/Value")
        
        right_scan[0] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg01/X/Sensor/Value")
        right_scan[1] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg03/X/Sensor/Value")
        right_scan[2] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg05/X/Sensor/Value")
        right_scan[3] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg07/X/Sensor/Value")
        right_scan[4] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg09/X/Sensor/Value")
        right_scan[5] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg11/X/Sensor/Value")
        right_scan[6] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg13/X/Sensor/Value")
        right_scan[7] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg14/X/Sensor/Value")
        right_scan[8] = memory_service.getData("Device/SubDeviceList/Platform/LaserSensor/Right/Horizontal/Seg15/X/Sensor/Value")
        print("left_scan="+str(left_scan))
        print("front_scan="+str(front_scan))
        print("right_scan="+str(right_scan))

        vLeft,vRight=braitenberg(left_scan,front_scan,right_scan)
        x=(vLeft+vRight)/2
        R=0.4# or R=5
        theta=(vRight-vLeft)/(2*R)
        y=x*math.tan(theta)
        print("x="+str(x),";y="+str(y)+";x+y="+str(x+y)+";theta="+str(theta))
        motion.post.moveTo(x,y,theta,_async=True)


        # Unsubscribe from sonars, this will stop sonars (at hardware level)
        # sonar_service.unsubscribe("myApplication")
        # id1 = motion.post.moveTo(0.1, 0.0, 0.0)#2 metre;  1 metre ; 0.5 = 90 degres
        # motion.wait(id1, 0)
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