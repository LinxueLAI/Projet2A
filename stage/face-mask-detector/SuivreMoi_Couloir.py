from robot import *
# import sys
import almath
from numpy import array
# -*- encoding: UTF-8 -*-

class Interaction:
    def __init__(self):
	    self.pepper = Pepper("192.168.2.169", 9559)
    
    def suivezMoi_Couloir(self):
        self.pepper.navigation_service.stopLocalization()
        # self.path="/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
        #loads the map from the directory where it was saved during exploration
        # self.path = "/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
        # self.path = "/home/nao/.local/share/Explorer/2015-09-23T205318.743Z.explo"
        # self.path = "/home/nao/.local/share/Explorer/2015-09-23T212450.001Z.explo"
        # self.path = "/home/nao/.local/share/Explorer/2015-09-28T203343.039Z.explo"
        self.path = "/home/nao/.local/share/Explorer/2015-09-28T210020.730Z.explo"
        self.pepper.navigation_service.loadExploration(str(self.path))
        self.pepper.navigation_service.startLocalization()
        #the map made by the robot has associated a coordinate system relative to the robot. the place where the robot starts the movement is the [0.0,0.0,0.0]. we define this position as our home
        self.home = self.pepper.navigation_service.getRobotPositionInMap()
        print "saved home position:"
        print self.home 

        self.pepper.motion_service.moveTo(0.0,0.0,3.1415926)
        self.pepper.pick_a_volunteer()
        # find a person, begin the walking process
        self.pepper.say("Suivez-moi s'il vous plait.")
        self.pepper.motion_service.moveTo(0.0,0.0,3.1415926)

        time_start = time.time()
        arrival = False
        p=array([[2.0,0.0],[4.0,0.0],[6.0,0.0],[8.0,0.0],[10.0,0.0],[12.0,0.0],[14.0,0.0]])#right -   ;left +
        i = 0
        self.pepper.motion_service.setAngles("HeadPitch", -0.1, 0.2)
        while i<7:
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
                # print "out"
                if i%2==0:
			self.pepper.say("Suivez-moi s'il vous plait.")
		else:
			self.pepper.say("C'est bien! continuer!")
                self.pepper.move_forward(0.0)
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
        self.pepper.motion_service.moveTo(0.0,0.0,-3.1415926)

        time_end = time.time()
        time_c = time_end - time_start
        print "time cost = "+str(time_c)+'s'
        
        self.pepper.say("vous avec marcher "+str(round((time.time()-time_start),1))+" seconds")
        self.pepper.say("j'ai fini la demonstration.")

Test1 = Interaction()
Test1.suivezMoi_Couloir()
print "terminating..."
