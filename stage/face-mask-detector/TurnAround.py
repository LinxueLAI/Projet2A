from robot import *
from numpy import *
import argparse
import sys
import time
import almath
pepper = Pepper("192.168.2.169", 9559)
pepper.posture_service.goToPosture("Stand", 0.5)
pepper.pick_a_volunteer()
pepper.say("suivez-moi")
PosX = 5
PosY = 0
theta = math.pi
pepper.motion_service.move(0, 0, theta)
time.sleep(4)

try:
    while 1:
        pepper.motion_service.move(1, 0, 0)
        time.sleep(4)
        pepper.motion_service.move(0, 0, theta)
        time.sleep(4)
        if (pepper.posture_service.getPostureFamily() == "Standing") :
            print "ok"
            pepper.motion_service.moveTo(-0.2,0.0,0.0)
            pepper.motion_service.moveTo(0.0,0.0,-0.4)
except KeyboardInterrupt:
    pepper.move_forward(0.0)
    pass
# pepper.posture_service.goToPosture("Stand", 0.5)
# pepper.navigation_service.stopLocalization()
# path = "/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
# pepper.navigation_service.loadExploration(str(path))
# pepper.navigation_service.startLocalization()
# home = pepper.navigation_service.getRobotPositionInMap()
# print "home position =  "+str(home[0])

# print "go to point a :"+str(0)+","+str(0)
# pepper.navigation_service.navigateToInMap([0.,0.,0.0])
# positionErrorThresholdPos = 0.1
# positionErrorThresholdAng = 0.03
# pepper.say("Suivez-moi s'il vous plait.")

# # pepper.motion_service.moveTo(0.0,0.0,3.14159)
# time_start = time.time()
# arrival = False
# p=array([[3.0,0.0],[3.0,-2.0],[0.0,-2.0],[0.0,0.0]])
# i = 0
# pepper.motion_service.setAngles("HeadPitch", -0.1, 0.2)
# while i<4:
#     print "i = "+str(i)
#     initPosition = almath.Pose2D(pepper.motion_service.getRobotPosition(True))
#     print "initPos = "+str(initPosition)
#     targetDistance = almath.Pose2D(int(p[i][0]),int(p[i][1]),0.0)
#     expectedEndPosition = initPosition * targetDistance
#     # pepper.motion_service.moveTo(3.0,0.0,0.0)
#     print "go to point"+str(p[i][0])+","+str(p[i][1])
#     pepper.navigation_service.navigateToInMap([int(p[i][0]),int(p[i][1]),0.0])
#     print "position in map = "+str(pepper.navigation_service.getRobotPositionInMap()[0])
#     realEndPosition = almath.Pose2D(pepper.motion_service.getRobotPosition(False))
#     positionError = realEndPosition.diff(expectedEndPosition)
#     print "positionError= "+str(positionError)
#     positionError.theta = almath.modulo2PI(positionError.theta)
#     pepper.motion_service.moveTo(0.0,0.0,-3.1415926)
#     try:   
#         arrived = False
#         time_2 = time.time()
#         # while (time.time()-time_2)<10:
#         while not arrived:
#             print "time_Cost = "+str(time.time()-time_2)
#             arrived = pepper.trackFace()
#         print "out"
#         pepper.move_forward(0.0)
#         pepper.motion_service.moveTo(0.0,0.0,3.1415926/2)
#     except KeyboardInterrupt:
#         pepper.move_forward(0.0)
#         pass
#     time.sleep(2)
#     print "now position = "+str(almath.Pose2D(pepper.motion_service.getRobotPosition(False)))
#     print "arrive at "+str(p[i][0])+","+str(p[i][1])

#     i = i + 1
# print "finished"

# time_end = time.time()
# time_c = time_end - time_start
# print "time cost = "+str(time_c)+'s'
# pepper.say("votre vitesse moyenne est "+str(round(10/(time.time()-time_start),2))+"metre par second")

