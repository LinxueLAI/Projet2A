from robot import *
# import config
# from robot_detect_image import *
from numpy import *
import argparse
import sys
import time
import almath
pepper = Pepper("192.168.2.169", 9559)
#pepper.autonomous_life_off()
# pepper.pick_a_volunteer()
pepper.posture_service.goToPosture("Stand", 0.5)
pepper.navigation_service.stopLocalization()
path = "/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
pepper.navigation_service.loadExploration(str(path))
pepper.navigation_service.startLocalization()
home = pepper.navigation_service.getRobotPositionInMap()
print "home position =  "+str(home[0])

print "go to point a :"+str(0)+","+str(0)
pepper.navigation_service.navigateToInMap([0.,0.,0.0])
# print "reached a :"+str(pepper.navigation_service.getRobotPositionInMap()[0])
# time_start = time.time()

# print "go to point b :"+str(2)+","+str(0)
# pepper.navigation_service.navigateToInMap([2.,0.,0.0])
# print "reached b :"+str(pepper.navigation_service.getRobotPositionInMap()[0])

# print "go to point c :"+str(2)+","+str(2)
# pepper.navigation_service.navigateToInMap([2.,2.,0.0])
# print "reached c :"+str(pepper.navigation_service.getRobotPositionInMap()[0])

# print "go to point d :"+str(0)+","+str(2)
# pepper.navigation_service.navigateToInMap([0.,2.,0.0])
# print "reached b :"+str(pepper.navigation_service.getRobotPositionInMap()[0])

# print "go to point e :"+str(0)+","+str(0)
# pepper.navigation_service.navigateToInMap([0.,0.,0.0])
# print "reached e :"+str(pepper.navigation_service.getRobotPositionInMap()[0])
# # time_start = time.time()

# print "go to point home :"+str(0)+","+str(0)
# pepper.navigation_service.navigateToInMap([home[0][0],home[0][1],0.0])
# print "reached home :"+str(pepper.navigation_service.getRobotPositionInMap()[0])

# pepper.move_forward(-0.1)
# time.sleep(10)
# pepper.motion_service.stopMove()
# time_end = time.time()
# time_c = time_end - time_start
# print "time cost = "+str(time_c)+'s'
# pepper.motion_service.moveTo(3.0,0.0,0.0)
positionErrorThresholdPos = 0.1
positionErrorThresholdAng = 0.03
pepper.say("Suivez-moi s'il vous plait.")

# pepper.motion_service.moveTo(0.0,0.0,3.14159)
time_start = time.time()
arrival = False
p=array([[3.0,0.0],[3.0,-2.0],[0.0,-2.0],[0.0,0.0]])
i = 0
pepper.motion_service.setAngles("HeadPitch", -0.1, 0.2)
while i<4:
    print "i = "+str(i)
    initPosition = almath.Pose2D(pepper.motion_service.getRobotPosition(True))
    print "initPos = "+str(initPosition)
    targetDistance = almath.Pose2D(int(p[i][0]),int(p[i][1]),0.0)
    expectedEndPosition = initPosition * targetDistance
    # pepper.motion_service.moveTo(3.0,0.0,0.0)
    print "go to point"+str(p[i][0])+","+str(p[i][1])
    pepper.navigation_service.navigateToInMap([int(p[i][0]),int(p[i][1]),0.0])
    print "position in map = "+str(pepper.navigation_service.getRobotPositionInMap()[0])
    realEndPosition = almath.Pose2D(pepper.motion_service.getRobotPosition(False))
    positionError = realEndPosition.diff(expectedEndPosition)
    print "positionError= "+str(positionError)
    positionError.theta = almath.modulo2PI(positionError.theta)
    pepper.motion_service.moveTo(0.0,0.0,-3.1415926)
    try:   
        arrived = False
        time_2 = time.time()
        # while (time.time()-time_2)<10:
        while not arrived:
            print "time_Cost = "+str(time.time()-time_2)
            arrived = pepper.trackFace()
        print "out"
        pepper.move_forward(0.0)
        pepper.motion_service.moveTo(0.0,0.0,3.1415926/2)
    except KeyboardInterrupt:
        pepper.move_forward(0.0)
        pass
    time.sleep(2)
    print "now position = "+str(almath.Pose2D(pepper.motion_service.getRobotPosition(False)))
    print "arrive at "+str(p[i][0])+","+str(p[i][1])
            # if pepper.posture_service.getPostureFamily() == "Standing" and pepper.motionProxy.robotIsWakeUp():
            # arrival = True
                # pepper.motion_service.moveTo(3.0,0.0,0.0)
        # else:
            # pepper.motion_service.moveTo(-0.2,0.0,0.0)
            # pepper.motion_service.moveTo(0.0,0.0,-0.4)
            # pepper.navigation_service.navigateToInMap([int(p[i][0]),int(p[i][1]),0.0])
    i = i + 1
print "finished"
# print "reached a :"+str(pepper.navigation_service.getRobotPositionInMap()[0])

# pepper.motion_service.moveTo(0.0,0.0,3.14159)
# try:
#     while 1:
#         print "time = "+str(time.time())
#         pepper.trackFace()
# except KeyboardInterrupt:
#     pepper.move_forward(0.0)
#     pass
time_end = time.time()
time_c = time_end - time_start
print "time cost = "+str(time_c)+'s'
pepper.say("votre vitesse moyenne est "+str(round(10/(time.time()-time_start),2))+"metre par second")
# try:
# pepper.move_forward(-2.0)#speed = 3.0
# except KeyboardInterrupt:
# time.sleep(3)
# pepper.move_forward(0.0)
# pepper.move_forward(-2.0)#speed = 3.0
