from robotDetector import *
from PIL import Image
from threading import Thread
import sys
import almath
import motion
import random
# -*- encoding: UTF-8 -*-

class Fios(Thread):
    def __init__(self,val):
	Thread.__init__(self)
	self.val=val
	self.pepper = Pepper("192.168.2.169", 9559)
	#self.pepper.autonomous_life_off()
	self.prototxtPath = os.path.sep.join(["face_detector", "deploy.prototxt"])
	self.weightsPath = os.path.sep.join(["face_detector","res10_300x300_ssd_iter_140000.caffemodel"])
	self.faceNet = cv2.dnn.readNet(self.prototxtPath, self.weightsPath)
	self.label = ""
	self.arrive=False
	# load the face mask detector model from disk
	print("[INFO] loading face mask detector model...")
	self.maskNet = load_model("mask_detector.model")
	self.pepper.subscribe_camera("camera_top", 2, 30)

    def videoStream(self):
	print("[INFO] loading face detector model...")

	# initialize the video stream and allow the camera sensor to warm up
	print("[INFO] starting video stream...")

	# loop over the frames from the video stream
	while True:
		frame = self.pepper.get_camera_frame(show=False)

		# detect faces in the frame and determine if they are wearing a face mask or not
		(locs, preds) = self.pepper.detect_and_predict_mask(frame, self.faceNet, self.maskNet)

		# loop over the detected face locations and their corresponding
		# locations
		for (box, pred) in zip(locs, preds):
			# unpack the bounding box and predictions
			(startX, startY, endX, endY) = box
			(mask, withoutMask) = pred

			# determine the class label and color we'll use to draw
			# the bounding box and text
			self.label = "Mask" if mask > withoutMask else "No Mask"
			color = (0, 255, 0) if self.label == "Mask" else (0, 0, 255)
			
			# include the probability in the label
			label1 = "{}: {:.2f}%".format(self.label, max(mask, withoutMask) * 100)

			# display the label and bounding box rectangle on the output
			# frame
			cv2.putText(frame, label1, (startX, startY - 10),
				cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
			cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)

		# show the output frame
		cv2.imshow("Frame", frame)
		self.label=""
		key = cv2.waitKey(1) & 0xFF

		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			break

	# do a bit of cleanup
	cv2.destroyAllWindows()

    def parole(self):
	nbHuman = 0
	nbMasque = 0

	while self.arrive ==False:
		self.pepper.tracker_service.registerTarget("Face", 0.15)
		self.pepper.tracker_service.setMode("Move")
		self.pepper.tracker_service.track("Face")
		frame = self.pepper.get_camera_frame(show=False)
		(locs, preds) = self.pepper.detect_and_predict_mask(frame, self.faceNet, self.maskNet)

		for (box, pred) in zip(locs, preds):
			# unpack the bounding box and predictions
			(startX, startY, endX, endY) = box
			(mask, withoutMask) = pred

			# determine the class label and color we'll use to draw
			# the bounding box and text
			self.label = "Mask" if mask > withoutMask else "No Mask"
			color = (0, 255, 0) if self.label == "Mask" else (0, 0, 255)
			
			# include the probability in the label
			label1 = "{}: {:.2f}%".format(self.label, max(mask, withoutMask) * 100)

			# display the label and bounding box rectangle on the output
			# frame
			cv2.putText(frame, label1, (startX, startY - 10),
				cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
			cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)
			m = random.randint(0,9)
			n = random.randint(0,100)
		if self.label=="No Mask":
			self.pepper.blink_eyes([255, 0, 0])
			self.pepper.tablet_service.showImage("https://png.pngtree.com/png-clipart/20200401/original/pngtree-hand-drawn-2019-new-corona-virus-wearing-a-mask-figure-png-image_5329325.jpg")
			#self.pepper.tablet_service.hideImage()
			nbHuman = nbHuman+1
			if m < 2:
				self.pepper.say("Portez votre masque si vous plait.")
			elif m < 4:
				self.pepper.say("N'oubliez pas de porter votre masque.")
			elif m < 6:
				self.pepper.say("Vous oubliez votre masque?")
			elif m < 8:
				self.pepper.say("N'oubliez pas votre masque.")
			else:
				self.pepper.say("Bonjour,vous devez porter votre masque.")
			print "image output"
			cv2.imwrite("./tmp/"+self.label+str(n)+".png", frame)
			print"I've met "+str(nbHuman)+" peoples,there're "+str(nbMasque)+" masks"
			time.sleep(2)
		elif self.label=="Mask":
			
			self.pepper.blink_eyes([0, 255, 0])
			nbHuman = nbHuman+1
			nbMasque = nbMasque+1
			self.pepper.say("C'est bien, vous portez votre masque.")
			print "image output"
			cv2.imwrite("./tmp/"+self.label+str(n)+".png", frame)
			print"I've met "+str(nbHuman)+" peoples,there're "+str(nbMasque)+" masks"		
			time.sleep(2)
		self.pepper.blink_eyes([0, 0, 255])
		self.label=""
		key = ""
			
		# if the `q` key was pressed, break from the loop
		#if key == ord("q"):
		#	break

    def def_point_show(self): #in def_point it is defined the movements needed in order to retrieve the desired coordinates of the map
        #if this funtion is to be run for any reason more then once in a row the location needs to be stopped 
        global a, b, c, d, e ,f, g, h
        self.pepper.navigation_service.stopLocalization()
        xfourpts = [0.0,2.0,2.0,0.0,0.0]
        yfourpts = [0.0,0.0,-2.0,-2.0,0.0]
        #two lists are created empty to later append the coordinates relative to X and Y of the coordinate system of the robot
        xlstpoints=[]
        ylstpoints=[]
        path="/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"#path
	#path="/home/nao/.local/share/Explorer/2015-07-15T174625.158Z.explo"
        #loads the map from the directory where it was saved during exploration
        self.pepper.navigation_service.loadExploration(str(path))
	guess =[0.,0.]
        self.pepper.navigation_service.relocalizeInMap(guess)
	print "Now it is relocated"
	#time.sleep(10)
	time.sleep(5)
        self.pepper.navigation_service.startLocalization()
        
        #the map made by the robot has associated a coordinate system relative to the robot. the place where the robot starts the movement is the [0.0,0.0,0.0]. we define this position as our home
        self.home = self.pepper.navigation_service.getRobotPositionInMap()
        print "saved home position"
	print "home="+str(self.home)
        
	#self.pepper.say("Je peux marcher.")
        #self.pepper.motion_service.moveTo(0.0,0.0,3.14159)
        #self.pepper.motion_service.moveTo(3.0,0.0,0.0)
        #self.pepper.navigation_service.navigateToInMap([3.,0.,0.])
        #self.a = self.pepper.navigation_service.getRobotPositionInMap() #a is defined in the same position as home. the reason for this is to follow a logic when it is called in the steps.
        #a=self.a 
        #print "point a ="+str(a)
	#time.sleep(5)
        #self.pepper.motion_service.moveTo(3.0,0.0,0.0)
        #self.pepper.navigation_service.navigateToInMap([3.,0.,0.])
        #self.b = self.pepper.navigation_service.getRobotPositionInMap() #a is defined in the same position as home. the reason for this is to follow a logic when it is called in the steps.
        #b=self.b 
        #print "point b ="+str(b)
	#time.sleep(5)
        #self.pepper.motion_service.moveTo(3.0,0.0,0.0)
        #self.pepper.navigation_service.navigateToInMap([3.,0.,0.])
        #self.c = self.pepper.navigation_service.getRobotPositionInMap() #a is defined in the same position as home. the reason for this is to follow a logic when it is called in the steps.
        #c=self.c
        #print "point c ="+str(c)
	#time.sleep(5)
        #self.pepper.motion_service.moveTo(0.0,0.0,3.14159)
	#time.sleep(10)
	#self.pepper.motion_service.moveTo(6.0,0.0,0.0)
        #self.pepper.navigation_service.navigateToInMap([3.,0.,0.])
        #self.b = self.pepper.navigation_service.getRobotPositionInMap() #a is defined in the same position as home. the reason for this is to follow a logic when it is called in the steps.
        #b=self.b 
        #print "point b ="+str(b)
        

	#self.pepper.navigation_service.navigateToInMap([self.c[0][0],self.c[0][1],0.])
	#print "arrive at c"
        #self.pepper.navigation_service.navigateToInMap([self.b[0][0],self.b[0][1],0.])
	#print "arrive at b"
	self.pepper.navigation_service.navigateToInMap([3.0,0.,0.])
	print "arrive at"+str(self.pepper.navigation_service.getRobotPositionInMap())
	self.pepper.navigation_service.navigateToInMap([self.home[0][0],self.home[0][1],0.])
	print "arrive at home"
	self.pepper.say("Je retourne au point d'origine")
	self.pepper.stand()
	self.pepper.motion_service.moveTo(0.0,0.0,3.1415926/2)

	time.sleep(5)
	self.pepper.navigation_service.navigateToInMap([0.,-4.0,0.])
	print "arrive at (0,-4)"
	self.pepper.navigation_service.navigateToInMap([self.home[0][0],self.home[0][1],0.])
	print "arrive at home"
	self.pepper.say("Je retourne au point d'origine")
	self.pepper.stand()
	self.pepper.motion_service.moveTo(0.0,0.0,-3.1415926/2)
	time.sleep(5)
	self.pepper.navigation_service.navigateToInMap([0.,4.0,0.])
	print "arrive at (0,4)"
	self.pepper.navigation_service.navigateToInMap([self.home[0][0],self.home[0][1],0.])
	print "arrive at home"
	self.pepper.say("C'est fini")	
	result_map = self.pepper.navigation_service.getMetricalMap()
	print"resolution="+str(result_map[0])
        map_width = result_map[1]
        map_height = result_map[2]
        img = np.array(result_map[4]).reshape(map_width, map_height)
        img = (100 - img) * 2.55 # from 0..100 to 255..0
        img = np.array(img, np.uint8)
        Image.frombuffer('L',  (map_width, map_height), img, 'raw', 'L', 0, 1).show()
    
    def def_pointSuivant(self): #in def_point it is defined the movements needed in order to retrieve the desired coordinates of the map
        #if this funtion is to be run for any reason more then once in a row the location needs to be stopped 
        self.pepper.navigation_service.stopLocalization()
	#self.path="/home/nao/.local/share/Explorer/2015-07-20T170449.116Z.explo"
	self.path="/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
        #loads the map from the directory where it was saved during exploration
        self.pepper.navigation_service.loadExploration(str(self.path))
        self.pepper.navigation_service.startLocalization()

	#result_map = self.pepper.navigation_service.getMetricalMap()
	#print "resolution="+str(result_map[0])
	#print "width ="+str(result_map[1])
	#print "height ="+str(result_map[2])
	#print "originOffsetXY ="+str(result_map[3])
	#print "pxlValue = "+str(type(result_map[4]))+",length= "+str(np.shape(result_map[4]))
        #map_width = result_map[1]
        #map_height = result_map[2]
        #img = np.array(result_map[4]).reshape(map_width, map_height)
        #img = (100 - img) * 2.55 # from 0..100 to 255..0
        #img = np.array(img, np.uint8)
	#print str(img)
        #Image.frombuffer('L',  (map_width, map_height), img, 'raw', 'L', 0, 1).show()

        #the map made by the robot has associated a coordinate system relative to the robot. the place where the robot starts the movement is the [0.0,0.0,0.0]. we define this position as our home
        self.home = self.pepper.navigation_service.getRobotPositionInMap()
        print "saved home position:"
        print self.home        
        self.pepper.motion_service.moveTo(0.0,0.0,0.0)
       
        self.a = self.pepper.navigation_service.getRobotPositionInMap() #a is defined in the same position as home. the reason for this is to follow a logic when it is called in the steps. 
        print "Saved position of A in map:"
        print self.a
        time.sleep(2)

        self.pepper.motion_service.moveTo(3.0,0.0,0.0)
        self.b = self.pepper.navigation_service.getRobotPositionInMap()
        print "Saved position of B in map:"
        print self.b
        time.sleep(5)

        # self.pepper.motion_service.moveTo(0.0,0.0,-3.1415926/2) #rotation of 90 in radians the minus sine make the rotation movement clockwise
        self.pepper.motion_service.moveTo(2.0,0.0,0.0) 
        self.c = self.pepper.navigation_service.getRobotPositionInMap()
        print "Saved position of C in map:"
        print self.c
        time.sleep(5)

        self.pepper.motion_service.moveTo(0.0,0.0,-3.1415926)
        self.pepper.posture_service.goToPosture("StandInit",0.5)
        self.pepper.motion_service.moveTo(3.0,0.0,0.0) 
        self.d = self.pepper.navigation_service.getRobotPositionInMap()
        print "Saved position of D in map:"
        print self.d
        time.sleep(5)

        # self.pepper.motion_service.moveTo(0.0,0.0,-3.1415926)
        self.pepper.motion_service.moveTo(2.0,0.0,0.0) 
        self.e = self.pepper.navigation_service.getRobotPositionInMap()
        print "Saved position of e in map:"
        print self.e
        time.sleep(2)
        
        self.pepper.navigation_service.navigateToInMap([self.home[0][0],self.home[0][1],0.0]) #after the definition of all the points it is used the navigation method for pepper to reach the home position. the orientation on arrival is not under control
        print "home:"
        print self.home
        
        self.pepper.posture_service.goToPosture("StandInit",0.5)
        return self.a, self.b, self.c, self.d, self.e
    
    def steps(self):
        time.sleep(5)
        #the steps followed to make the behaviour. first the robot goes to 3 defined points close to each other to make adjustments to the orientation    
        #then the robot starts moving autonomously to the target
        print "go to point a : "+str(self.a[0][0])+","+str(self.a[0][1])
        self.pepper.navigation_service.navigateToInMap([self.a[0][0],self.a[0][1],0.0])
	print "reached a:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
        print "go to point b : "+str(self.b[0][0])+","+str(self.b[0][1])
        self.pepper.navigation_service.navigateToInMap([self.b[0][0],self.b[0][1],0.0])
	print "reached b:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
        print "go to point c : "+str(self.c[0][0])+","+str(self.c[0][1])
        self.pepper.navigation_service.navigateToInMap([self.c[0][0],self.c[0][1],0.0])
	print "reached c:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
        print "go to point d : "+str(self.d[0][0])+","+str(self.d[0][1])
        self.pepper.navigation_service.navigateToInMap([self.d[0][0],self.d[0][1],0.0])
	print "reached d:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
        print "go to point e : "+str(self.e[0][0])+","+str(self.e[0][1])
        self.pepper.navigation_service.navigateToInMap([self.e[0][0],self.e[0][1],0.0])
	print "reached e:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
        
        #the robot then returns home to rest
        self.pepper.navigation_service.navigateToInMap([self.home[0][0],self.home[0][1],0.0])
	print "reached home:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
        self.pepper.posture_service.goToPosture("StandInit",0.5)

    def navigateToPoint(self):
        self.pepper.navigation_service.stopLocalization()
	self.path="/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
        self.pepper.navigation_service.loadExploration(str(self.path))

        self.pepper.navigation_service.startLocalization()

        time.sleep(5)

        self.home = self.pepper.navigation_service.getRobotPositionInMap()
        print "saved home position:"+str(self.home[0])
	tours = 1
	while tours > 0:
		print "go to point a : "+str(0)+","+str(0)
		self.pepper.navigation_service.navigateToInMap([0.,0.,0.0])
		print "reached a:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		print "go to point b : "+str(3)+","+str(0)
		self.pepper.navigation_service.navigateToInMap([3.,0.,0.0])
		print "reached b:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		print "go to point c : "+str(5)+","+str(0)
		self.pepper.navigation_service.navigateToInMap([5.,0.,0.0])
		print "reached c:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		#print "go to point d : "+str(7)+","+str(0)
		#self.pepper.navigation_service.navigateToInMap([7.,0.,0.0])
		#print "reached d:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		#print "go back to point c : "+str(5)+","+str(0)
		#self.pepper.navigation_service.navigateToInMap([5.,0.,0.0])
		#print "reached c:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		print "go to point b1 : "+str(5)+","+str(1)
		self.pepper.navigation_service.navigateToInMap([5.,1.,0.0])
		print "reached b1:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		#print "go to point c1 : "+str(0)+","+str(5)
		#self.pepper.navigation_service.navigateToInMap([0.,5.,0.0])
		#print "reached c1:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		print "go to point d1 : "+str(5)+","+str(2)
		self.pepper.navigation_service.navigateToInMap([5.,2.,0.0])
		print "reached d1:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		print "go to point e1 : "+str(5)+","+str(0)
		self.pepper.navigation_service.navigateToInMap([5.,0.,0.0])
		print "reached e1:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		print "go to point d : "+str(2)+","+str(0)
		self.pepper.navigation_service.navigateToInMap([2.,0.,0.0])
		print "reached d:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		print "go to point e : "+str(0)+","+str(0)
		self.pepper.navigation_service.navigateToInMap([0.,0.,0.0])
		print "reached e:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		#the robot then returns home to rest
		print "go to home : "+str(self.home[0][0])+","+str(self.home[0][1])
        	self.pepper.navigation_service.navigateToInMap([self.home[0][0],self.home[0][1],0.0])
		print "reached home:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		tours = tours - 1
        self.pepper.posture_service.goToPosture("StandInit",0.5)
	self.arrive = True
	print "arrive="+str(self.arrive)
    def setDialog(self):
	    self.pepper.dialog_service.setLanguage("French")
#75
	    # writing topics' qichat code as text strings (end-of-line characters are important!)
	    topic_content_1 = ('topic: ~example_topic_content()\n'
		               'language: frf\n'
		               'concept:(aliments) [fruits poulet oeufs boeuf]\n'
		               'proposal: veuillez me parler\n'
		               'u: (Bonjour) Bonjour humain.\n'
		               'u: (Je [veux "voudrais"] {quelques} _~aliments) Daccord! Vous devez vraiment aimer les $1 .\n'
		               'u: (comment vas tu aujourd hui) Je vais bien merci, et toi?\n'
		               'u: (ca va?) Super!\n'
		               'u: (Pepper) Oui. Voua avez des questions?\n'
		               'u: (Jai deja porter le masque) Bien!\n'
		               'u: (quest-ce que tu mange?) Je suis un robot, je nai pas besoin de manger les aliments. Jai besoin de seulement electricite.\n'
		               'u: (As-tu bien dormi?) Non! Tu as oublie de me couper et me charger!\n'
		               'u: (posez-moi une question) aimez-vous le poisson? u1: (oui) ce est bon pour la sante u1: (non) je prefere la viande.\n'
		               'u: (parlez des animaux) avez-vous un chat ou un chien? u1: (jai un chat) super u1: (jai un chien) je prefere un chat u1: (non) ah. dommage.\n'
		               'u: (mon prenom est _*) Bonjour $name=$1\n'
		               'u: (Quel est mon prenom?) ^first["votre prenom est $name" "je ne sais pas"] u1:(oui) rara u1: (non) OK ^clear(name)\n'
		               'u: (Quelle est votre position?) ma position est ^break ^call(ALRobotPosture.getPosture()) c1:(_*) $1\n'
		               'u: ([e:FrontTactilTouched e:MiddleTactilTouched e:RearTactilTouched]) Tu as touche ma tette\n'
		               'u: ([e:faceDetected "Salut"]) Salut humain!\n')

	    topic_content_2 = ('topic: ~dummy_topic()\n'
		               'language: frf\n'
		               'u:(test) [a b "c d" "e f g"]\n')

	    # # Loading the topics directly as text strings
	    topic_name_1 = self.pepper.dialog_service.loadTopicContent(topic_content_1)
	    topic_name_2 = self.pepper.dialog_service.loadTopicContent(topic_content_2)

	    # # Activating the loaded topics
	    self.pepper.dialog_service.activateTopic(topic_name_1)
	    self.pepper.dialog_service.activateTopic(topic_name_2)

	    # # Starting the dialog engine - we need to type an arbitrary string as the identifier
	    # # We subscribe only ONCE, regardless of the number of topics we have activated
	    self.pepper.dialog_service.subscribe('my_dialog_example')

    def run(self):
	if self.val == 1:
		# Ensure that the tablet wifi is enable
		self.pepper.tablet_service.enableWifi()
		self.setDialog()
		key = ""
	#self.pepper.autonomous_life_off()
	#self.pepper.motion_service.wakeUp()
		#self.def_pointSuivant()
		#self.pepper.explore(4.0)
		#while True:
		#	if key == ord("q"):
		#		break
		self.pepper.posture_service.goToPosture("StandInit",0.5)				
		#self.pepper.remoteTerminal()
		try:
			self.navigateToPoint()		
			#self.steps()
		#self.videoStream()
			raw_input("\nSpeak to the robot using rules from both the activated topics. Press Enter when finished:")
		finally:
			print 'ok'
			# stopping the dialog engine
			self.pepper.dialog_service.unsubscribe('my_dialog_example')

			# Deactivating all topics
			self.pepper.dialog_service.deactivateTopic(topic_name_1)
			self.pepper.dialog_service.deactivateTopic(topic_name_2)

			# now that the dialog engine is stopped and there are no more activated topics,
			# we can unload all topics and free the associated memory
			self.pepper.dialog_service.unloadTopic(topic_name_1)
			self.pepper.dialog_service.unloadTopic(topic_name_2)
		#self.def_point_show()
		#self.steps_show()
	if self.val == 2:
		self.parole()

myThread1 = Fios(1)
myThread1.setName('Thread 1')

myThread2 = Fios(2)
myThread2.setName('Thread 2')
myThread2.daemon = True



print "Initiating Threads"
myThread1.start()
myThread2.start()

myThread1.join()
myThread2.join()
print "main Terminating..."

