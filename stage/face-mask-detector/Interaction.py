from robot import *
from PIL import Image
from threading import Thread
import sys
import almath
import motion
import random
from numpy import array
# -*- encoding: UTF-8 -*-

class Interaction:
    def __init__(self):
	    self.pepper = Pepper("192.168.2.169", 9559)
    
    def navigateToPoint(self):
        self.pepper.navigation_service.stopLocalization()
        # self.path="/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
        # self.path = "/home/nao/.local/share/Explorer/2015-09-18T183258.341Z.explo"
        self.path = "/home/nao/.local/share/Explorer/2015-09-18T210133.604Z.explo"
        self.pepper.navigation_service.loadExploration(str(self.path))

        self.pepper.navigation_service.startLocalization()
        time.sleep(1)
        self.home = self.pepper.navigation_service.getRobotPositionInMap()
        print "saved home position:"+str(self.home[0])
        # self.pepper.getBehaviors(self.pepper.behavior_mng_service)
        print "maintenant le robot va commencer un petit talk......"
        self.pepper.motion_service.setAngles("HeadPitch", -0.1, 0.2)
        #self.pepper.motion_service.moveTo(0.0,0.0,3.1415926)
        #self.pepper.pick_a_volunteer()
        #self.pepper.launchAndStopBehavior(self.pepper.behavior_mng_service, 'smalltalk-e525f6/behavior_1')
        time.sleep(2)
        tours = 1
        while tours > 0:

            self.suivezMoi()
            # self.pepper.motion_service.moveTo(0.0,0.0,-3.1415926)# -: turn to right; +: turn to left
            tours = tours - 1
            #the robot then returns home to rest
        print "go to home : "+str(self.home[0][0])+","+str(self.home[0][1])
        self.pepper.navigation_service.navigateToInMap([self.home[0][0],self.home[0][1],0.0])
        print "reached home:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])

    def suivezMoi(self):
        self.pepper.say("Suivez-moi s'il vous plait.")

        # pepper.motion_service.moveTo(0.0,0.0,3.14159)
        time_start = time.time()
        arrival = False
        p=array([[3.0,0.0],[3.0,2.0],[0.0,2.0],[0.0,0.0]])#right -   ;left +
        i = 0
        self.pepper.motion_service.setAngles("HeadPitch", -0.1, 0.2)
        while i<4:
            print "i = "+str(i)
            initPosition = almath.Pose2D(self.pepper.motion_service.getRobotPosition(True))
            print "initPos = "+str(initPosition)
            targetDistance = almath.Pose2D(int(p[i][0]),int(p[i][1]),0.0)
            expectedEndPosition = initPosition * targetDistance
            # pepper.motion_service.moveTo(3.0,0.0,0.0)
            print "go to point"+str(p[i][0])+","+str(p[i][1])
            self.pepper.navigation_service.navigateToInMap([int(p[i][0]),int(p[i][1]),0.0])
            print "position in map = "+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
            realEndPosition = almath.Pose2D(self.pepper.motion_service.getRobotPosition(False))
            positionError = realEndPosition.diff(expectedEndPosition)
            print "positionError= "+str(positionError)
            positionError.theta = almath.modulo2PI(positionError.theta)
            self.pepper.motion_service.moveTo(0.0,0.0,3.1415926)
            try:   
                arrived = False
                time_2 = time.time()
                # while (time.time()-time_2)<10:
                while not arrived:
                    print "time_Cost = "+str(time.time()-time_2)
                    arrived = self.pepper.trackFace()
                print "out"
                self.pepper.move_forward(0.0)
                self.pepper.motion_service.moveTo(0.0,0.0,-3.1415926/2)
            except KeyboardInterrupt:
                self.pepper.move_forward(0.0)
                pass
            if(positionError.x>0.3) or (positionError.y>0.3):
                print "continuer atteindre le point "+str(p[i][0])+","+str(p[i][1])
                self.pepper.navigation_service.navigateToInMap([int(p[i][0]),int(p[i][1]),0.0])
                print "position in map = "+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
            time.sleep(2)
            print "now position = "+str(almath.Pose2D(self.pepper.motion_service.getRobotPosition(False)))
            print "arrive at "+str(p[i][0])+","+str(p[i][1])
            i = i + 1
        print "finished"

        time_end = time.time()
        time_c = time_end - time_start
        print "time cost = "+str(time_c)+'s'
        # self.pepper.say("votre vitesse moyenne est "+str(round(10/(time.time()-time_start),2))+"metre par second")
        self.pepper.say("j'ai fini la demonstration.")

    def run(self):
		#self.pepper.autonomous_life_off()
		#self.pepper.motion_service.wakeUp()
		self.pepper.posture_service.goToPosture("StandInit",0.5)				
		try:
			self.navigateToPoint()		
			self.pepper.tablet_service.hideImage()
			#self.videoStream()
		finally:
			print 'ok'

Test1 = Interaction()
Test1.run()
print "terminating..."
