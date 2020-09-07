import time
from naoqi import ALProxy
import numpy
from PIL import Image
import argparse
import sys
import matplotlib.pyplot as plt

class Scan():
    def __init__(self): # __init__ is the first thing to run when the program is started
        #in the function __init__ we call every module that is going to be used throughout the present script.
        ip = "192.168.2.169"
        self.tts = ALProxy("ALTextToSpeech", ip, 9559)
        self.motion = ALProxy("ALMotion", ip, 9559)
        self.posture = ALProxy("ALRobotPosture", ip, 9559)
        self.tracker = ALProxy("ALTracker", ip, 9559)
        self.memory = ALProxy("ALMemory",ip,9559)
        # self.perception = ALProxy("ALPeoplePerception", ip, 9559)
        self.autonomous_life_service = ALProxy("ALAutonomousLife",ip,9559)
        self.navigation = ALProxy("ALNavigation",ip,9559)
        self.face_detection = ALProxy("ALFaceDetection",ip,9559)
        self.face_characteristic = ALProxy("ALFaceCharacteristics",ip,9559)
        self.battery = ALProxy("ALBattery",ip,9559)
        self.led_service = ALProxy("ALLeds",ip,9559)
        
        #posture standinit gives a more natural position for the robot. instead of being looking to the sky it looks in the direction of the horizon
        self.posture.goToPosture("StandInit",0.5)

        #Since in this script we making a movement with the actuators, it is good practice to set the stiffness to the maximum in order for the robot to keep the final position of the actuators in the end of the movement.        
        stiffnesses  = 1.0
        names ="Body"
        self.motion.setStiffnesses(names, stiffnesses)
    
    # some behaviors: say,dance, find a humian,shake hands...
    def say(self, text):
        self.tts.say(text)
        print("[INFO]: Robot says: " + text)
        
    def dance(self,motion):
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([1.52, 2.36, 3.32, 4.16, 5.08, 5.92, 6.88, 7.72, 8.16, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 16.24])
        keys.append([-0.476475, 0.338594, -0.476475, 0.338594, -0.476475, 0.338594, -0.476475, 0.338594, 0.0680678, -0.476475, 0.338594, -0.476475, 0.338594, -0.476475, 0.338594, -0.476475, 0.338594, -0.17185])

        names.append("HeadYaw")
        times.append([1.52, 2.36, 3.32, 4.16, 5.08, 5.92, 6.88, 7.72, 8.16, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 16.24])
        keys.append([-0.745256, 0.0411095, -0.745256, 0.0411095, -0.745256, 0.018508, -0.745256, 0.289725, 0.425684, 0.745256, -0.0411095, 0.745256, -0.0411095, 0.745256, -0.018508, 0.745256, -0.289725, 0.00916195])

        names.append("HipPitch")
        times.append([0.68, 1.52, 2.36, 3.32, 4.16, 5.08, 5.92, 6.88, 7.72, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 16.24])
        keys.append([-0.376033, -0.036954, -0.344024, -0.0404086, -0.339835, -0.038321, -0.341769, -0.0367355, -0.34817, -0.035085, -0.341769, -0.0382761, -0.339629, -0.0396041, -0.341605, -0.0362713, -0.343065, -0.0495279])

        names.append("HipRoll")
        times.append([1.52, 2.36, 3.32, 4.16, 5.08, 5.92, 6.88, 7.72, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 16.24])
        keys.append([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        names.append("KneePitch")
        times.append([0.68, 1.52, 2.36, 3.32, 4.16, 5.08, 5.92, 6.88, 7.72, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 16.24])
        keys.append([0.166965, -0.00379234, 0.185949, -0.0129339, 0.180821, -0.00320919, 0.187035, -0.00931236, 0.182162, -0.0111253, 0.187035, -0.00683206, 0.184441, -0.0119436, 0.179202, -0.0114876, 0.187691, -0.013167])

        names.append("LElbowRoll")
        times.append([0.68, 1.04, 1.48, 2.32, 3.28, 4.12, 5.04, 5.88, 6.84, 7.68, 8.12, 8.48, 8.8, 9.2, 9.64, 10.12, 10.6, 11, 11.44, 11.92, 12.36, 12.76, 13.2, 13.68, 14.16, 14.56, 15, 15.6, 16.2, 16.4])
        keys.append([-1.37289, -1.12923, -0.369652, -0.202446, -0.369652, -0.202446, -0.369652, -0.202446, -0.369652, -0.202446, -0.820305, -0.23305, -0.138102, -1.309, -0.257754, -1.4591, -0.138102, -1.309, -0.257754, -1.4591, -0.138102, -1.309, -0.257754, -1.4591, -0.138102, -1.309, -0.257754, -0.984366, -0.513992, -0.424876])

        names.append("LElbowYaw")
        times.append([0.68, 1.48, 2.32, 3.28, 4.12, 5.04, 5.88, 6.84, 7.68, 8.12, 8.48, 8.8, 9.2, 9.64, 10.12, 10.6, 11, 11.44, 11.92, 12.36, 12.76, 13.2, 13.68, 14.16, 14.56, 15, 15.6, 16.2, 16.4])
        keys.append([-0.65506, -0.380475, -0.618244, -0.380475, -0.618244, -0.380475, -0.618244, -0.380475, -0.618244, 0.410152, 0.818273, 0.851412, 0.0750492, 0.00157596, 0.460767, 0.851412, 0.0750492, 0.00157596, 0.460767, 0.851412, 0.0750492, 0.00157596, 0.460767, 0.851412, 0.0750492, 0.00157596, -1.34565, -1.22484, -1.21037])

        names.append("LHand")
        times.append([0.68, 1.04, 1.48, 2.32, 3.28, 4.12, 5.04, 5.88, 6.84, 7.68, 8.48, 8.8, 9.2, 9.64, 10.12, 10.6, 11, 11.44, 11.92, 12.36, 12.76, 13.2, 13.68, 14.16, 14.56, 15, 15.6, 16.2, 16.4])
        keys.append([0.2, 0.6, 0.2648, 0.264, 0.2648, 0.264, 0.2648, 0.264, 0.2648, 0.264, 0.663802, 0.928, 0.3, 0.0283999, 0.75, 0.928, 0.3, 0.0283999, 0.75, 0.928, 0.3, 0.0283999, 0.75, 0.928, 0.3, 0.5284, 0.936396, 0.950347, 0.2968])

        names.append("LShoulderPitch")
        times.append([0.68, 1.48, 2.32, 3.28, 4.12, 5.04, 5.88, 6.84, 7.68, 8.12, 8.48, 8.8, 9.64, 10.6, 11.44, 12.36, 13.2, 14.16, 15, 16.4])
        keys.append([0.97784, 1.29573, 1.40466, 1.29573, 1.40466, 1.29573, 1.40466, 1.29573, 1.40466, 0.172788, -1.04904, -1.19188, 0.995607, -1.19188, 0.995607, -1.19188, 0.995607, -1.19188, 0.995607, 1.47106])

        names.append("LShoulderRoll")
        times.append([0.68, 1.48, 2.32, 3.28, 4.12, 5.04, 5.88, 6.84, 7.68, 8.48, 8.8, 9.2, 9.64, 10.12, 10.6, 11, 11.44, 11.92, 12.36, 12.76, 13.2, 13.68, 14.16, 14.56, 15, 15.6, 16.2])
        keys.append([0.500047, 0.401871, 0.35585, 0.401871, 0.35585, 0.401871, 0.35585, 0.401871, 0.35585, 0.886453, 0.966481, 1.23332, 0.324005, 1.23332, 0.966481, 1.23332, 0.324005, 1.23332, 0.966481, 1.23332, 0.324005, 1.23332, 0.966481, 1.23332, 0.324005, 0.407503, 0.146991])

        names.append("LWristYaw")
        times.append([0.68, 1.04, 1.48, 2.32, 3.28, 4.12, 5.04, 5.88, 6.84, 7.68, 8.48, 8.8, 9.64, 10.6, 11.44, 12.36, 13.2, 14.16, 15, 16.2, 16.4])
        keys.append([0.11961, -0.289725, -0.395814, -0.420357, -0.395814, -0.420357, -0.395814, -0.420357, -0.395814, -0.420357, -0.122946, -0.107338, -0.400331, -0.107338, -0.400331, -0.107338, -0.400331, -0.107338, -0.400331, 0.000370312, 0.0827939])

        names.append("RElbowRoll")
        times.append([0.68, 1.08, 1.52, 1.92, 2.36, 2.84, 3.32, 3.72, 4.16, 4.64, 5.08, 5.48, 5.92, 6.4, 6.88, 7.28, 7.72, 8.52, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 15.64, 16.24, 16.44])
        keys.append([1.34689, 1.1205, 0.138102, 1.309, 0.257754, 1.4591, 0.138102, 1.309, 0.257754, 1.4591, 0.138102, 1.309, 0.257754, 1.4591, 0.138102, 1.309, 0.257754, 0.372085, 0.369652, 0.202446, 0.369652, 0.202446, 0.369652, 0.202446, 0.369652, 0.202446, 0.82205, 0.519567, 0.429562])

        names.append("RElbowYaw")
        times.append([0.68, 1.08, 1.52, 1.92, 2.36, 2.84, 3.32, 3.72, 4.16, 4.64, 5.08, 5.48, 5.92, 6.4, 6.88, 7.28, 7.72, 8.52, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 15.64, 16.24, 16.44])
        keys.append([0.59515, 0.567232, -0.851412, -0.0750492, -0.00157596, -0.460767, -0.851412, -0.0750492, -0.00157596, -0.460767, -0.851412, -0.0750492, -0.00157596, -0.460767, -0.851412, -0.0750492, -0.00157596, 0.352279, 0.380475, 0.618244, 0.380475, 0.618244, 0.380475, 0.618244, 0.380475, 0.618244, 1.26711, 1.23132, 1.21028])

        names.append("RHand")
        times.append([0.68, 1.08, 1.52, 1.92, 2.36, 2.84, 3.32, 3.72, 4.16, 4.64, 5.08, 5.48, 5.92, 6.4, 6.88, 7.28, 7.72, 8.52, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 16.24, 16.44])
        keys.append([0.2, 0.95, 0.928, 0.3, 0.0283999, 0.75, 0.928, 0.3, 0.0283999, 0.75, 0.928, 0.3, 0.0283999, 0.75, 0.928, 0.3, 0.5284, 0.271478, 0.2648, 0.264, 0.2648, 0.264, 0.2648, 0.264, 0.2648, 0.264, 0.596785, 0.2976])

        names.append("RShoulderPitch")
        times.append([0.68, 1.52, 2.36, 3.32, 4.16, 5.08, 5.92, 6.88, 7.72, 8.52, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 16.24])
        keys.append([0.915841, -1.19188, 0.995607, -1.19188, 0.995607, -1.19188, 0.995607, -1.19188, 0.995607, 1.281, 1.29573, 1.40466, 1.29573, 1.40466, 1.29573, 1.40466, 1.29573, 1.40466, 1.47268])

        names.append("RShoulderRoll")
        times.append([0.68, 1.08, 1.52, 1.92, 2.36, 2.84, 3.32, 3.72, 4.16, 4.64, 5.08, 5.48, 5.92, 6.4, 6.88, 7.28, 7.72, 8.52, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 15.64, 16.44])
        keys.append([-0.905123, -1.30837, -0.966481, -1.23332, -0.324005, -1.23332, -0.966481, -1.23332, -0.324005, -1.23332, -0.966481, -1.23332, -0.324005, -1.23332, -0.966481, -1.23332, -0.324005, -0.397371, -0.401871, -0.35585, -0.401871, -0.35585, -0.401871, -0.35585, -0.401871, -0.35585, -0.310669, -0.174533])

        names.append("RWristYaw")
        times.append([0.68, 1.52, 2.36, 3.32, 4.16, 5.08, 5.92, 6.88, 7.72, 8.52, 8.84, 9.68, 10.64, 11.48, 12.4, 13.24, 14.2, 15.04, 16.24, 16.44])
        keys.append([-0.401949, 0.107338, 0.400331, 0.107338, 0.400331, 0.107338, 0.400331, 0.107338, 0.400331, 0.391888, 0.395814, 0.420357, 0.395814, 0.420357, 0.395814, 0.420357, 0.395814, 0.420357, 0.00501826, 0.108872])

        self.motion.angleInterpolation(names, keys, times, True)

    def unsubscribe_effector(self):
        self.tracker.unregisterAllTargets()
        self.tracker.setEffector("None")
        print("[INFO]: End-effector is unsubscribed")
    
    def turn_around(self, speed):
        self.motion.move(0, 0, speed)
    
    def stand(self):
        self.posture.goToPosture("Stand", 0.5)
        print("[INFO]: Robot is in default position")  

    def stop_moving(self):
        self.motion.stopMove()

    def pick_a_human(self):
        volunteer_found = False
        self.unsubscribe_effector()
        self.stand()
        print("[INFO]: Robot is in default position")
        self.say("Je cherche un humain.")
        proxy_name = "FaceDetection" + str(numpy.random)
        print("[INFO]: Pick a volunteer mode started")

        while not volunteer_found:
            wait = numpy.random.randint(500, 1500) / 1000
            theta = numpy.random.randint(-10, 10)
            self.turn_around(theta)
            time.sleep(wait)
            self.stop_moving()
            self.stand()
            self.face_detection.subscribe(proxy_name, 500, 0.0)
            for memory in range(5):
                time.sleep(0.5)
                output = self.memory.getData("FaceDetected")
                print("...")
                if output and isinstance(output, list) and len(output) >= 2:
                    print("Face detected")
                    volunteer_found = True

        self.say("Je trouvais un humain! salut!")
        self.stand()
        try:
            self.tracker.registerTarget("Face", 0.15)
            self.tracker.setMode("Move")
            self.tracker.track("Face")
            self.tracker.setEffector("RArm")
            self.get_face_properties()

        finally:
            time.sleep(2)
            self.unsubscribe_effector()
            self.stand()
            self.face_detection.unsubscribe(proxy_name)

    def autonomous_life_on(self):
        self.autonomous_life_service.setState("interactive")
        print("[INFO]: Autonomous life is on")
    
    def autonomous_life_off(self):
        self.autonomous_life_service.setState("disabled")
        self.stand()
        print("[INFO]: Autonomous life is off")

    def get_face_properties(self):
        self.autonomous_life_on()
        # emotions = ["neutral", "happy", "surprised", "angry", "sad"]
        emotions = ["neutre", "heureux", "surpris", "en colere", "triste"]
        face_id = self.memory.getData("PeoplePerception/PeopleList")
        recognized = None
        try:
            recognized = self.face_characteristic.analyzeFaceCharacteristics(face_id[0])
        except Exception as error:
            print("[ERROR]: Cannot find a face to analyze.")
            self.say("Je ne peux pas reconnaitre ce visage.")

        if recognized:
            properties = self.memory.getData("PeoplePerception/Person/" + str(face_id[0]) + "/ExpressionProperties")
            gender = self.memory.getData("PeoplePerception/Person/" + str(face_id[0]) + "/GenderProperties")
            age = self.memory.getData("PeoplePerception/Person/" + str(face_id[0]) + "/AgeProperties")

            # Gender properties
            if gender[1] > 0.4:
                if gender[0] == 0:
                    self.say("Bonjour Mademoiselle!")
                elif gender[0] == 1:
                    self.say("Bonjour Monsieur!")
            else:
                self.say("Bonjour humain!")

            # Age properties
            if gender[1] == 1:
                self.say("Vous avez " + str(int(age[0])) + "ans.")
            else:
                self.say("vous ressemblez a " + str(int(age[0])) + " non, j'ai dit " + str(int(age[0]-5)))

            # Emotion properties
            emotion_index = (properties.index(max(properties)))

            if emotion_index > 0.5:
                self.say("Je suis sur que votre emotion est " + emotions[emotion_index])
            else:
                self.say("Je suppose que votre emotion est " + emotions[emotion_index])
    
    def battery_status(self):
        """Say a battery status"""
        battery = self.battery.getBatteryCharge()
        self.say("J'ai " + str(battery) + " pour cent de battery")
    def shake_hand(self):
        self.stand()
        # Raise a hand to human
        speed = 0.3
        self.motion.angleInterpolationWithSpeed(["RShoulderPitch", "RWristYaw", "RHand"], [0.8, 2.5, 1.0], speed)
        # Wait to touch a hand
        while True:
            try:
                status = self.memory.getData("HandRightBackTouched")
                if status:
                    self.motion.angleInterpolationWithSpeed("RHand", 0.0, speed*3)
                    break
            except KeyboardInterrupt:
                break
        # Get hand down
        self.motion.angleInterpolationWithSpeed(["RShoulderPitch", "RWristYaw", "RHand"], [3.5, 0.00, 0.0], 1.0)#1.0=speed max?
        # Reset position
        self.stand()
    def blink_eyes(self, rgb):
        self.led_service.fadeRGB('AllLeds', rgb[0], rgb[1], rgb[2], 1.0)

    def explore(self, radius, force=False):
        self.motion.wakeUp() #it is only used if the robot is in rest position.
        # Explore the environement
        # radius = 3.0
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
        print "now the robot: it has arrived the initial position "
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
        global a, b, c, d, e ,f, g, h
        self.navigation.stopLocalization()
        xfourpts = [0.0,2.0,2.0,0.0,0.0]
        yfourpts = [0.0,0.0,-2.0,-2.0,0.0]
        #two lists are created empty to later append the coordinates relative to X and Y of the coordinate system of the robot
        xlstpoints=[]
        ylstpoints=[]
        path="/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"#path
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
        self.motion.moveTo(1.0,0.0,0.0) #20 centimeters in front
        self.b = self.navigation.getRobotPositionInMap()
        b=self.b
        
        bx=self.b[0][0]
        xlstpoints.append(bx) #apppend the value of x coordinate in point b as the second value of the list
        by=self.b[0][1]
        ylstpoints.append(by)
        print "Saved position of B in map"
        time.sleep(1)
    
        self.motion.moveTo(1.0,0.0,0.0)
        self.c = self.navigation.getRobotPositionInMap()
        c=self.c
        
        cx=self.c[0][0]
        xlstpoints.append(cx) 
        cy=self.c[0][1]
        ylstpoints.append(cy)
        print "Saved position of C in map"
        time.sleep(1)
        
        self.motion.moveTo(0.0,0.0,-3.1415926/2)#rotation of 180 in radians the minus sine make the rotation movement clockwise
        self.motion.moveTo(1.0,0.0,0.0)#after the rotation move 1.3m in front to save the position in the other direction
        self.d = self.navigation.getRobotPositionInMap()
        d=self.d
        
        dx=self.d[0][0]
        xlstpoints.append(dx)
        dy=self.d[0][1]
        ylstpoints.append(dy)
        print "Saved position of D in map"
        time.sleep(1)
        
        self.motion.moveTo(1.0,0.0,0.0)
        self.e = self.navigation.getRobotPositionInMap()
        e=self.e
        
        ex=self.e[0][0]
        xlstpoints.append(ex)
        ey=self.e[0][1]
        ylstpoints.append(ey)
        print "Saved position of E in map"
        time.sleep(1)

        self.motion.moveTo(0.0,0.0,-3.1415926/2)#rotation of 180 in radians the minus sine make the rotation movement clockwise
        self.motion.moveTo(1.0,0.0,0.0)
        self.f = self.navigation.getRobotPositionInMap()
        f=self.f
        
        fx=self.f[0][0]
        xlstpoints.append(fx)
        fy=self.f[0][1]
        ylstpoints.append(fy)
        print "Saved position of F in map"
        time.sleep(1)

        self.motion.moveTo(1.0,0.0,0.0)
        self.g = self.navigation.getRobotPositionInMap()
        g=self.g
        
        gx=self.g[0][0]
        xlstpoints.append(gx)
        gy=self.g[0][1]
        ylstpoints.append(gy)
        print "Saved position of G in map"
        time.sleep(1)

        self.motion.moveTo(0.0,0.0,-3.1415926/2)#rotation of 180 in radians the minus sine make the rotation movement clockwise
        self.motion.moveTo(2.0,0.0,0.0)

        self.h = self.navigation.getRobotPositionInMap()
        h=self.h
        
        hx=self.h[0][0]
        xlstpoints.append(hx)
        hy=self.h[0][1]
        ylstpoints.append(hy)
        print "home \n" 
        
        print "liste de photos x"
        print (xlstpoints)
        print "liste de photos y"
        print (ylstpoints)
        
        #plot the coordinates in a graphic that shows the path taken in relation to the coordinates of the robot
        plt.title('Result Analysis')
        plt.plot(xfourpts, yfourpts, 'g',label ="theoretical path")
        plt.plot(xlstpoints, ylstpoints, 'r',label ="actual path")
        plt.legend() 
        plt.axis([-1.0, 2.5, -2.5, 0.5])
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.show()
        
        self.posture.goToPosture("StandInit",0.5)
        return self.home, self.a, self.b, self.c, self.d, self.e, self.f,self.g 
    
    def steps_show(self):
                
        print "initiated behaviour"
        time.sleep(5)
        self.navigation.navigateToInMap([self.a[0][0],self.a[0][1],0.0])
        #the steps followed to make the behaviour. first the robot goes to 3 defined points close to each other to make adjustments to the orientation
        print "arrive at point a, say something:"
        self.say("Bonjour, je m'appelle Pepper et je ferai quelques demonstrations.")
        self.say("Je peux marcher.")
        self.navigation.navigateToInMap([self.b[0][0],self.b[0][1],0.0])
        self.navigation.navigateToInMap([self.c[0][0],self.c[0][1],0.0])
        print "arrive at point c, say something:"
        self.say("Je suis au point C maintenant, je danse.")
        self.dance(self.motion)
        time.sleep(1)

        self.navigation.navigateToInMap([self.d[0][0],self.d[0][1],0.0])
        self.navigation.navigateToInMap([self.e[0][0],self.e[0][1],0.0])
        print "arrive at point e, say something:"
        self.say("Je suis au point E maintenant.Je veux vous serrer la main.")
        self.shake_hand()
        time.sleep(1)
        
        self.navigation.navigateToInMap([self.f[0][0],self.f[0][1],0.0])
        self.navigation.navigateToInMap([self.g[0][0],self.g[0][1],0.0])
        print "arrive at point g, say something:"
        self.say("Je suis au point G maintenant.Je change le couleur des yeux.")
        self.blink_eyes([255, 0, 0])
        time.sleep(1)

        self.navigation.navigateToInMap([self.a[0][0],self.a[0][1],0.0])
        time.sleep(1)
        # self.navigation.navigateToInMap([self.home[0][0],self.home[0][1],0.0])
        print "arrive at point a, say something:"
        self.say("Je suis retourne a l'endroit d'origine.")
        self.pick_a_human()# pick a humain

        self.say("Ma demonstration est terminee.")  
        self.posture.goToPosture("StandInit",0.5)

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
        self.motion.moveTo(0.4,0.0,0.0) #40 centimeters in front
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

    def def_pointSuivant(self): #in def_point it is defined the movements needed in order to retrieve the desired coordinates of the map
        #if this funtion is to be run for any reason more then once in a row the location needs to be stopped 
        self.navigation.stopLocalization()
        # self.path="/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
        #loads the map from the directory where it was saved during exploration
        self.path = "/home/nao/.local/share/Explorer/2015-09-18T210133.604Z.explo"
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

        self.motion.moveTo(0.0,0.0,3.1415926/2) #rotation of 90 in radians the minus sine make the rotation movement clockwise
        self.motion.moveTo(2.0,0.0,0.0) 
        self.c = self.navigation.getRobotPositionInMap()
        print "Saved position of C in map:"
        print self.c
        time.sleep(2)

        self.motion.moveTo(0.0,0.0,3.1415926/2)
        self.motion.moveTo(3.0,0.0,0.0) 
        self.d = self.navigation.getRobotPositionInMap()
        print "Saved position of D in map:"
        print self.d
        time.sleep(2)

        self.motion.moveTo(0.0,0.0,3.1415926/2)
        self.motion.moveTo(2.0,0.0,0.0) 
        self.e = self.navigation.getRobotPositionInMap()
        print "Saved position of e in map:"
        print self.e
        time.sleep(2)
        self.motion.moveTo(0.0,0.0,3.1415926/2)
        
        self.navigation.navigateToInMap([self.home[0][0],self.home[0][1],0.0]) #after the definition of all the points it is used the navigation method for pepper to reach the home position. the orientation on arrival is not under control
        print "home:"
        print self.navigation.getRobotPositionInMap()
        
        self.posture.goToPosture("StandInit",0.5)
        return self.a, self.b, self.c, self.d, self.e
    

    def steps_with_behaviors(self):
        self.navigation.navigateToInMap([self.a[0][0],self.a[0][1],0.0])
        print "arrive at point a, say something:"
        self.say("Bonjour, je m'appelle Pepper et je ferai quelques demonstrations.")
        self.say("Je peux marcher.")
        time.sleep(2)
        self.navigation.navigateToInMap([self.b[0][0],self.b[0][1],0.0])
        print "arrive at point b, say something:"
        self.say("Je suis au point B maintenant, je danse.")
        self.dance(self.motion)
        time.sleep(2)
        self.navigation.navigateToInMap([self.c[0][0],self.c[0][1],0.0])
        print "arrive at point c, say something:"
        self.say("Je suis au point C maintenant.Je veux vous serrer la main.")
        self.shake_hand()

        self.navigation.navigateToInMap([self.d[0][0],self.d[0][1],0.0])
        print "arrive at point d, say something:"
        self.say("Je suis au point D maintenant.Je change le couleur des yeux.")
        self.blink_eyes([255, 0, 0])
        
        self.navigation.navigateToInMap([self.a[0][0],self.a[0][1],0.0])
        print "arrive at point e(a), say something:"
        self.say("Je suis retourne a l'endroit d'origine.")
        self.pick_a_human()# pick a humain

        self.say("Ma demonstration est terminee.")
        self.posture.goToPosture("StandInit",0.5)

    def steps(self):
        time.sleep(5)
        #the steps followed to make the behaviour. first the robot goes to 3 defined points close to each other to make adjustments to the orientation    
        #then the robot starts moving autonomously to the target
        print "go to point a : "+str(self.a[0][0])+","+str(self.a[0][1])
        self.navigation.navigateToInMap([self.a[0][0],self.a[0][1],0.0])
        print "arrive at a:"+str(self.navigation.getRobotPositionInMap())

        print "go to point b : "+str(self.b[0][0])+","+str(self.b[0][1])
        self.navigation.navigateToInMap([self.b[0][0],self.b[0][1],0.0])
        print "arrive at b:"+str(self.navigation.getRobotPositionInMap())

        print "go to point c : "+str(self.c[0][0])+","+str(self.c[0][1])
        self.navigation.navigateToInMap([self.c[0][0],self.c[0][1],0.0])
        print "arrive at c:"+str(self.navigation.getRobotPositionInMap())

        print "go to point d : "+str(self.d[0][0])+","+str(self.d[0][1])
        self.navigation.navigateToInMap([self.d[0][0],self.d[0][1],0.0])
        print "arrive at d:"+str(self.navigation.getRobotPositionInMap())

        print "go to point e : "+str(self.e[0][0])+","+str(self.e[0][1])
        self.navigation.navigateToInMap([self.e[0][0],self.e[0][1],0.0])
        print "arrive at e:"+str(self.navigation.getRobotPositionInMap())

        
        #the robot then returns home to rest
        self.navigation.navigateToInMap([self.home[0][0],self.home[0][1],0.0])
        print "arrive at home:"+str(self.navigation.getRobotPositionInMap())
        self.posture.goToPosture("StandInit",0.5)


    def stepcarre(self):
        self.navigation.navigateToInMap([self.a[0][0],self.a[0][1],0.0])
        self.navigation.navigateToInMap([self.b[0][0],self.b[0][1],0.0])
        self.navigation.navigateToInMap([self.c[0][0],self.c[0][1],0.0])
        self.navigation.navigateToInMap([self.d[0][0],self.d[0][1],0.0])
        self.navigation.navigateToInMap([self.e[0][0],self.e[0][1],0.0])
        self.posture.goToPosture("StandInit",0.5)

    def steps2(self):
        self.navigation.navigateToInMap([0.0,0.0,0.0])
        print "arrive at home"
        self.navigation.navigateToInMap([0.0,2.0,0.0])
        print "arrive at a"
        self.navigation.navigateToInMap([2.0,2.0,0.0])
        print "arrive at b"
        self.navigation.navigateToInMap([2.0,0.0,0.0])
        print "arrive at c"
        self.navigation.navigateToInMap([0.0,0.0,0.0])
        print "arrive at d"
        self.navigation.navigateToInMap([0.0,2.0,0.0])
        print "arrive at e"
        print "once again:"
        self.navigation.navigateToInMap([0.0,2.0,0.0])
        print "arrive at a"
        self.navigation.navigateToInMap([2.0,2.0,0.0])
        print "arrive at b"
        self.navigation.navigateToInMap([2.0,0.0,0.0])
        print "arrive at c"
        self.navigation.navigateToInMap([0.0,0.0,0.0])
        print "arrive at d:"
        self.navigation.navigateToInMap([0.0,2.0,0.0])
        print "arrive at e:"
        # the robot then returns home to rest
        # self.navigation.navigateToInMap([self.home[0][0],self.home[0][1],0.0])
        # print "arrive at home:"
        # print self.home
        self.posture.goToPosture("StandInit",0.5)
        print "steps finished"

# Test1:
pepper = Scan()
# pepper.battery_status()
pepper.motion.setOrthogonalSecurityDistance(0.2)
# pepper.motion_service.setOrthogonalSecurityDistance(0.1)
print("OrthogonalSecurityDistance=")
print(pepper.motion.getOrthogonalSecurityDistance())#0.4
print("TangentialSecurityDistance=")
print(pepper.motion.getTangentialSecurityDistance())#0.1
# After the exploration the robot will stop in a random position
# time_start = time.time()
# pepper.motion.stopMove() 
# pepper.autonomous_life_off()

# path = pepper.navigation.saveExploration()
# print "Exploration saved at path: \"" + path + "\""
# pepper.navigation.stopExploration()
pepper.explore(15.0)#radius
print "exploration is finished"

# pepper.motion.setAngles("HeadPitch", -0.1, 0.2)
# print "1"
# pepper.motion.setAngles("HeadYaw", -0.5, 0.2)

# print "1"
# pepper.motion.setAngles("HeadYaw", 0.0, 0.2)

# print "2"
# pepper.motion.setAngles("HeadYaw", 0.5, 0.2)
# time.sleep(1)
# pepper.motion.setAngles("HeadYaw", -0.3, 0.2)
# print "2"
# time.sleep(1)
# pepper.motion.setAngles("HeadYaw", -0.5, 0.2)
# print "3"
# time.sleep(1)
# pepper.motion.setAngles("HeadYaw", -0.3, 0.2)
# print "1"
# time.sleep(1)
# pepper.motion.setAngles("HeadYaw", -0.1, 0.2)
# print "2"
# time.sleep(1)
# pepper.motion.setAngles("HeadYaw", 0.1, 0.2)
# print "3"
# time.sleep(1)
# result_map = pepper.navigation.getMetricalMap()
# map_width = result_map[1]
# map_height = result_map[2]
# img = numpy.array(result_map[4]).reshape(map_width, map_height)
# img = (100 - img) * 2.55 # from 0..100 to 255..0
# img = numpy.array(img, numpy.uint8)
# Image.frombuffer('L',  (map_width, map_height), img, 'raw', 'L', 0, 1).show()
# pepper.posture.goToPosture("StandInit",0.5)
# time_end = time.time()
# time_c = time_end - time_start
# print "time cost of exploration = "+str(time_c)+'s'

# pepper.motion.wakeUp()

# print "wait for 10 sec"
# time.sleep(10)

# pepper.navigation.navigateToInMap([0.0,0.0,0.0])
# pepper.navigation.navigateToInMap([0.5,0.0,0.0])
# pepper.navigation.navigateToInMap([1.0,0.0,0.0])

# pepper.def_pointSuivant()
# pepper.steps()

# pepper.def_point_show()
# pepper.steps_show()
# first=pepper.def_point_carre()
# print first # return self.home, self.a, self.b, self.c, self.d, self.e
# print "now the robot try to follow some steps:"
# # pepper.stepcarre()
# pepper.steps_with_behaviors()

# 	-second def-point. Change the coordinates and the number of points that you see fit for your application. see if the path taken by the robot is satisfatory
# second=pepper.def_point2()
# print second
# after the definition of points the robot navigates to the home-position. see if the robot stops relatively close to the spot where he began the definition of points
# print "now the robot try to follow some steps:"
# pepper.steps2() # put an obstacle in his path to see if he can avoid it or not 


