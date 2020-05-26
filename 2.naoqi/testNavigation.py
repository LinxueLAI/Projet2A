import time
from naoqi import ALProxy
import numpy
from PIL import Image
import argparse
import sys

class Scan():
    def __init__(self): # __init__ is the first thing to run when the program is started
        #in the function __init__ we call every module that is going to be used throughout the present script.
        ip = "192.168.2.169"
        # self.tts = ALProxy("ALTextToSpeech", ip, 9559)
        self.motion = ALProxy("ALMotion", ip, 9559)
        self.posture = ALProxy("ALRobotPosture", ip, 9559)
        # self.tracker = ALProxy("ALTracker", ip, 9559)
        # self.memory = ALProxy("ALMemory","172.16.0.77",9559)
        # self.perception = ALProxy("ALPeoplePerception", ip, 9559)
        self.navigation = ALProxy("ALNavigation",ip,9559)
    
        #posture standinit gives a more natural position for the robot. instead of being looking to the sky it looks in the direction of the horizon
        self.posture.goToPosture("StandInit",0.5)

        #Since in this script we making a movement with the actuators, it is good practice to set the stiffness to the maximum in order for the robot to keep the final position of the actuators in the end of the movement.        
        stiffnesses  = 1.0
        names ="Body"
        self.motion.setStiffnesses(names, stiffnesses)

    def explore(self, force=False):
      
        self.motion.wakeUp() #it is only used if the robot is in rest position.

        # Explore the environement, in a radius of 3 m.
        radius = 3.0
        error = self.navigation.explore(radius)
        if error != 0:
            print "Exploration failed."
            return
        # Saves the exploration on the robots disk
        self.path = self.navigation.saveExploration()
        print "Exploration saved at path: \"" + self.path + "\""
        # Start localization to navigate in map
        print "now the robot: startLocalisation "
        self.navigation.startLocalization()
        # Come back to initial position
        print "now the robot: it begins to go to initial position "
        self.navigation.navigateToInMap([0., 0., 0.])
        # Stop localization
        print "now the robot: ie has arrived the initial position "
        self.navigation.stopLocalization()
        # Retrieve and display the map built by the robot
        result_map = self.navigation.getMetricalMap()
        map_width = result_map[1]
        map_height = result_map[2]
        img = numpy.array(result_map[4]).reshape(map_width, map_height)
        img = (100 - img) * 2.55 # from 0..100 to 255..0
        img = numpy.array(img, numpy.uint8)
        Image.frombuffer('L',  (map_width, map_height), img, 'raw', 'L', 0, 1).show()
        self.posture.goToPosture("StandInit",0.5)

    def def_point_show(self): #in def_point it is defined the movements needed in order to retrieve the desired coordinates of the map
        #if this funtion is to be run for any reason more then once in a row the location needs to be stopped 
        global a, b, c, d, e ,f, h
        
        self.navigation.stopLocalization()
        
        #two lists are created empty to later append the coordinates relative to X and Y of the coordinate system of the robot
        xlstpoints=[]
        ylstpoints=[]
        
        #loads the map from the directory where it was saved during exploration
        self.navigation.loadExploration(str(path))
        self.navigation.startLocalization()
        
        #the map made by the robot has associated a coordinate system relative to the robot. the place where the robot starts the movement is the [0.0,0.0,0.0]. we define this position as our home
        self.home = self.navigation.getRobotPositionInMap()
        print "saved home position"
        
        self.motion.moveTo(0.0,0.0,0.0)
       
        self.a = self.navigation.getRobotPositionInMap() #a is defined in the same position as home. the reason for this is to follow a logic when it is called in the steps.
        a=self.a 
        
        ax=self.a[0][0] #gets the x coordinate of the coordinate system of the map
        xlstpoints.append(ax) #appends the coordinate to the empty list, saving it as the first value
        ay=self.a[0][1] #gets the y coordinate of the coordinate system of the map
        ylstpoints.append(ay) #appends the coordinate to the empty list, saving it as the first value
        print "Saved position of A in map"
        time.sleep(2)
        
        #since the navigation method as a deprecated coordinate Z we are not able to use navigation to define the points themselfs. we use the ALMotion module instead that give us high precision of movement to retrieve the coordinates.
        self.motion.moveTo(0.2,0.0,0.0) #20 centimeters in front
        self.b = self.navigation.getRobotPositionInMap()
        b=self.b
        
        bx=self.b[0][0]
        xlstpoints.append(bx) #apppend the value of x coordinate in point b as the second value of the list
        by=self.b[0][1]
        ylstpoints.append(by)
        print "Saved position of B in map"
        time.sleep(2)
    
        self.motion.moveTo(0.2,0.0,0.0)
        self.c = self.navigation.getRobotPositionInMap()
        c=self.c
        
        cx=self.c[0][0]
        xlstpoints.append(cx) 
        cy=self.c[0][1]
        ylstpoints.append(cy)
        print "Saved position of C in map"
        time.sleep(2)
        
        self.motion.moveTo(0.0,0.0,-3.1415)#rotation of 180 in radians the minus sine make the rotation movement clockwise
        self.motion.moveTo(1.3,0.0,0.0)#after the rotation move 1.3m in front to save the position in the other direction
        self.d = self.navigation.getRobotPositionInMap()
        d=self.d
        
        dx=self.d[0][0]
        xlstpoints.append(dx)
        dy=self.d[0][1]
        ylstpoints.append(dy)
        print "Saved position of D in map"
        time.sleep(2)
        
        self.motion.moveTo(0.2,0.0,0.0)
        self.e = self.navigation.getRobotPositionInMap()
        e=self.e
        
        ex=self.e[0][0]
        xlstpoints.append(ex)
        ey=self.e[0][1]
        ylstpoints.append(ey)
        print "Saved position of E in map"
        time.sleep(2)
      
        self.motion.moveTo(0.2,0.0,0.0)
        self.f = self.navigation.getRobotPositionInMap()
        f=self.f
        
        fx=self.f[0][0]
        xlstpoints.append(fx)
        fy=self.f[0][1]
        ylstpoints.append(fy)
        print "Saved position of F in map"
        time.sleep(2)
        
        self.navigation.navigateToInMap([self.home[0][0],self.home[0][1],0.0])  #after the definition of all the points it is used the navigation method for pepper to reach the home position. the orientation on arrival is not under control
        self.h = self.navigation.getRobotPositionInMap()
        h=self.h
        
        hx=self.h[0][0]
        xlstpoints.append(hx)
        hy=self.h[0][1]
        ylstpoints.append(hy)
        print "home \n" 
        
        print "lista de pontos x"
        print (xlstpoints)
        print "lista de pontos y"
        print (ylstpoints)
        
        #plot the coordinates in a graphic that shows the path taken in relation to the coordinates of the robot
        plt.plot(xlstpoints, ylstpoints, 'r')
        plt.axis([-2.0, 1.5, -0.8, 0.5])
        plt.show()
        
        self.posture.goToPosture("StandInit",0.5)
        return self.home, self.a, self.b, self.c, self.d, self.e, self.f 

    def def_point(self): #in def_point it is defined the movements needed in order to retrieve the desired coordinates of the map
        #if this funtion is to be run for any reason more then once in a row the location needs to be stopped 
        self.navigation.stopLocalization()
        
        #loads the map from the directory where it was saved during exploration
        self.navigation.loadExploration(str(self.path))
        self.navigation.startLocalization()

        #the map made by the robot has associated a coordinate system relative to the robot. the place where the robot starts the movement is the [0.0,0.0,0.0]. we define this position as our home
        self.home = self.navigation.getRobotPositionInMap()
        print "saved home position"
        
        self.motion.moveTo(0.0,0.0,0.0)
        print "now the robot is in [0,0,0]"
       
        self.a = self.navigation.getRobotPositionInMap() #a is defined in the same position as home. the reason for this is to follow a logic when it is called in the steps. 
        print "Saved position of A in map:"
        print self.a
        time.sleep(2)

        #since the navigation method as a coordinate Z deprecated we are not able to use navigation to define the points themselfs. we use the ALMotion module instead that give us high precision of movement to retrieve the coordinates.
        self.motion.moveTo(0.4,0.0,0.0) #20 centimeters in front
        # self.motion.moveTo(1.0,0.0,0.0)
        self.b = self.navigation.getRobotPositionInMap()
        print "Saved position of B in map:"
        print self.b
        time.sleep(2)
    
        self.motion.moveTo(0.4,0.0,0.0)
        self.c = self.navigation.getRobotPositionInMap()
        print "Saved position of C in map:"
        print self.c
        time.sleep(2)
        
        self.motion.moveTo(0.0,0.0,-3.1415) #rotation of 180 in radians the minus sine make the rotation movement clockwise
        self.motion.moveTo(1.8,0.0,0.0) #after the rotation move 1.3m in front to save the position in the other direction
        self.d = self.navigation.getRobotPositionInMap()
        print "Saved position of D in map:"
        print self.d

        time.sleep(2)
        
        self.motion.moveTo(0.4,0.0,0.0)   
        self.e = self.navigation.getRobotPositionInMap()
        print "Saved position of E in map:"
        print self.e
        time.sleep(2)
      
        self.motion.moveTo(0.4,0.0,0.0)
        self.f = self.navigation.getRobotPositionInMap()
        print "Saved position of F in map:"
        print self.f
        time.sleep(2)
        
        self.navigation.navigateToInMap([self.home[0][0],self.home[0][1],0.0]) #after the definition of all the points it is used the navigation method for pepper to reach the home position. the orientation on arrival is not under control
        print "home:"
        print self.home

        
        self.posture.goToPosture("StandInit",0.5)
        return self.home, self.a, self.b, self.c, self.d, self.e, self.f  

    def def_point2(self): #in def_point it is defined the movements needed in order to retrieve the desired coordinates of the map
        #if this funtion is to be run for any reason more then once in a row the location needs to be stopped 
        self.navigation.stopLocalization()
        
        #loads the map from the directory where it was saved during exploration
        self.navigation.loadExploration(str(self.path))
        self.navigation.startLocalization()

        #the map made by the robot has associated a coordinate system relative to the robot. the place where the robot starts the movement is the [0.0,0.0,0.0]. we define this position as our home
        self.home = self.navigation.getRobotPositionInMap()
        print "saved home position:"
        print self.home        
        self.motion.moveTo(0.0,0.0,0.0)
       
        self.a = self.navigation.getRobotPositionInMap() #a is defined in the same position as home. the reason for this is to follow a logic when it is called in the steps. 
        print "Saved position of A in map:"
        print self.a
        time.sleep(2)

        self.motion.moveTo(3.0,0.0,0.0)
        self.b = self.navigation.getRobotPositionInMap()
        print "Saved position of B in map:"
        print self.b
        time.sleep(2)

        self.motion.moveTo(0.0,0.0,-3.1415926/2) #rotation of 90 in radians the minus sine make the rotation movement clockwise
        self.motion.moveTo(2.0,0.0,0.0) 
        self.c = self.navigation.getRobotPositionInMap()
        print "Saved position of C in map:"
        print self.c
        time.sleep(2)

        self.motion.moveTo(0.0,0.0,-3.1415926/2)
        self.motion.moveTo(3.0,0.0,0.0) 
        self.d = self.navigation.getRobotPositionInMap()
        print "Saved position of D in map:"
        print self.d
        time.sleep(2)

        self.motion.moveTo(0.0,0.0,-3.1415926/2)
        self.motion.moveTo(2.0,0.0,0.0) 
        self.e = self.navigation.getRobotPositionInMap()
        print "Saved position of D in map:"
        print self.e
        time.sleep(2)
        
        self.navigation.navigateToInMap([self.home[0][0],self.home[0][1],0.0]) #after the definition of all the points it is used the navigation method for pepper to reach the home position. the orientation on arrival is not under control
        print "home:"
        print self.home
        
        self.posture.goToPosture("StandInit",0.5)
        return self.home, self.a, self.b, self.c, self.d, self.e  

    def steps(self):
        time.sleep(5)
        #the steps followed to make the behaviour. first the robot goes to 3 defined points close to each other to make adjustments to the orientation
        self.navigation.navigateToInMap([self.d[0][0],self.d[0][1],0.0])
        self.navigation.navigateToInMap([self.e[0][0],self.e[0][1],0.0])
        # self.navigation.navigateToInMap([self.f[0][0],self.f[0][1],0.0])
        
        #then the robot starts moving autonomously to the target
        self.navigation.navigateToInMap([self.a[0][0],self.a[0][1],0.0])
        self.navigation.navigateToInMap([self.b[0][0],self.b[0][1],0.0])
        self.navigation.navigateToInMap([self.c[0][0],self.c[0][1],0.0])
        
        
        #the robot then returns home to rest
        self.navigation.navigateToInMap([self.home[0][0],self.home[0][1],0.0])
        self.posture.goToPosture("StandInit",0.5)

    def stepcarre(self):
        self.navigation.navigateToInMap([self.a[0][0],self.a[0][1],0.0])
        self.navigation.navigateToInMap([self.b[0][0],self.b[0][1],0.0])
        self.navigation.navigateToInMap([self.c[0][0],self.c[0][1],0.0])
        self.navigation.navigateToInMap([self.d[0][0],self.d[0][1],0.0])
        self.navigation.navigateToInMap([self.e[0][0],self.e[0][1],0.0])
        self.posture.goToPosture("StandInit",0.5)

    def steps2(self):
        # time.sleep(5)

        #then the robot starts moving autonomously to the target
        # self.navigation.navigateToInMap([self.a[0][0],self.a[0][1],0.0])
        self.navigation.navigateToInMap([0.0,2.0,0.0])
        # print "wait for 10s"
        # time.sleep(10)
        print "arrive at a"
        # print self.a
        self.navigation.navigateToInMap([2.0,2.0,0.0])
        # self.motion.moveTo(0.0,0.0,-3.1415926/2)
        print "arrive at b"
        # print self.b

        self.navigation.navigateToInMap([2.0,0.0,0.0])
        # self.motion.moveTo(0.0,0.0,-3.1415926/2)

        print "arrive at c"
        # print self.c

        #the steps followed to make the behaviour. first the robot goes to 3 defined points close to each other to make adjustments to the orientation
        self.navigation.navigateToInMap([0.0,0.0,0.0])
        # self.motion.moveTo(0.0,0.0,-3.1415926/2)

        print "arrive at d"
        # print self.d

        self.navigation.navigateToInMap([0.0,2.0,0.0])
        # self.motion.moveTo(0.0,0.0,-3.1415926/2)

        print "arrive at e"
        print "once again:"
        # print self.e
        self.navigation.navigateToInMap([0.0,2.0,0.0])
        # print "wait for 10s"
        # time.sleep(10)
        print "arrive at a"
        # print self.a
        self.navigation.navigateToInMap([2.0,2.0,0.0])
        # self.motion.moveTo(0.0,0.0,-3.1415926/2)
        print "arrive at b"
        # print self.b

        self.navigation.navigateToInMap([2.0,0.0,0.0])
        # self.motion.moveTo(0.0,0.0,-3.1415926/2)

        print "arrive at c"
        # print self.c

        #the steps followed to make the behaviour. first the robot goes to 3 defined points close to each other to make adjustments to the orientation
        self.navigation.navigateToInMap([0.0,0.0,0.0])
        # self.motion.moveTo(0.0,0.0,-3.1415926/2)

        print "arrive at d:"
        # print self.d

        self.navigation.navigateToInMap([0.0,2.0,0.0])
        # self.motion.moveTo(0.0,0.0,-3.1415926/2)

        print "arrive at e:"
        #the robot then returns home to rest
        # self.navigation.navigateToInMap([self.home[0][0],self.home[0][1],0.0])
        # print "arrive at home:"
        # print self.home
        self.posture.goToPosture("StandInit",0.5)
        print "steps finished"

pepper = Scan()
pepper.motion.setOrthogonalSecurityDistance(0.2)
# pepper.motion_service.setOrthogonalSecurityDistance(0.1)
print("OrthogonalSecurityDistance=")
print(pepper.motion.getOrthogonalSecurityDistance())#0.4
print("TangentialSecurityDistance=")
print(pepper.motion.getTangentialSecurityDistance())#0.1
#After the exploration the robot will stop in a random position
pepper.explore()
print "exploration is finished"
#before running the definition of points in map, orientate the robot to the best orientation for your application. 
# This orientation can be made by hand. it will not affect the coordinate system
#running the definition of points in map
# print "now it is sleeping for 10 sec"
# time.sleep(10)
print "wait for 10 sec"
time.sleep(10)
first=pepper.def_point2()
print first # return self.home, self.a, self.b, self.c, self.d, self.e
print "now the robot try to follow some steps:"
pepper.stepcarre()
# 	-second def-point. Change the coordinates and the number of points that you see fit for your application. see if the path taken by the robot is satisfatory
# second=pepper.def_point2()
# print second
# after the definition of points the robot navigates to the home-position. see if the robot stops relatively close to the spot where he began the definition of points
# print "now the robot try to follow some steps:"
# pepper.steps2() # put an obstacle in his path to see if he can avoid it or not 


