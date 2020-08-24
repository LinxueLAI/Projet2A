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
	self.arrive=1
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
	while self.arrive == 1:
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
				#self.pepper.tablet_service.showImage("https://png.pngtree.com/png-clipart/20200401/original/pngtree-hand-drawn-2019-new-corona-virus-wearing-a-mask-figure-png-image_5329325.jpg")
				s = self.pepper.tablet_service.showImage("/home/nao/video/masque.jpg")
				print "s = "+str(s)
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

    def navigateToPoint(self):
        self.pepper.navigation_service.stopLocalization()
	self.path="/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
        self.pepper.navigation_service.loadExploration(str(self.path))

        self.pepper.navigation_service.startLocalization()

        time.sleep(5)
		
        self.home = self.pepper.navigation_service.getRobotPositionInMap()
        print "saved home position:"+str(self.home[0])
        self.pepper.getBehaviors(self.pepper.behavior_mng_service)
	print "maintenant le robot va commencer un petit talk......"
        self.pepper.launchAndStopBehavior(self.pepper.behavior_mng_service, 'smalltalk-e525f6/behavior_1')
	tours = 1
	while tours > 0:
		print "go to point a : "+str(0)+","+str(0)
		self.pepper.navigation_service.navigateToInMap([0.,0.,0.0])
		print "reached a:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		print "go to point b : "+str(3)+","+str(0)
		self.pepper.navigation_service.navigateToInMap([3.,0.,0.0])
		print "reached b:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
		self.pepper.getBehaviors(self.pepper.behavior_mng_service)
        	print "maintenant le robot va faire des mouvements......"
        	self.pepper.launchAndStopBehavior(self.pepper.behavior_mng_service, 'troismovements-c012e8/behavior_1')
		# self.pepper.say("Pourriez-vous faire la meme mouvement comme moi?")
		# time.sleep(1)
		# self.pepper.posture_service.goToPosture("StandInit",0.5)
		# self.pepper.wave(self.pepper.motion_service)
		print "go to point c : "+str(5)+","+str(0)
		self.pepper.navigation_service.navigateToInMap([5.,0.,0.0])
		print "reached c:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])

		self.pepper.say("Nous pouvons danser ensemble!")
		time.sleep(1)
		self.pepper.posture_service.goToPosture("StandInit",0.5)
		self.pepper.dance(self.pepper.motion_service)

		print "go to point b1 : "+str(5)+","+str(1)
		self.pepper.navigation_service.navigateToInMap([5.,1.,0.0])
		print "reached b1:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])

		print "go to point d1 : "+str(5)+","+str(2)
		self.pepper.navigation_service.navigateToInMap([5.,2.,0.0])
		print "reached d1:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])

		print "go to point e1 : "+str(5)+","+str(0)
		self.pepper.navigation_service.navigateToInMap([5.,0.,0.0])
		print "reached e1:"+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
        	self.pepper.getBehaviors(self.pepper.behavior_mng_service)
        	print "maintenant le robot va montrer un quiz simple......"
        	self.pepper.launchAndStopBehavior(self.pepper.behavior_mng_service, 'quizsimple-ef922a/behavior_1')
		# self.pepper.say("Pourriez-vous faire la meme mouvement comme moi?")
		# time.sleep(1)
		# self.pepper.posture_service.goToPosture("StandInit",0.5)
		# self.pepper.moveBody(self.pepper.motion_service)

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
        	self.pepper.getBehaviors(self.pepper.behavior_mng_service)
        	print "maintenant le robot va montrer un quiz memoire......"
        	self.pepper.launchAndStopBehavior(self.pepper.behavior_mng_service, 'quizsimple-ef922a/behavior_quiz')
		# self.arrive = True
		# print "arrive="+str(self.arrive)

    def run(self):
	if self.val == 1:
		# Ensure that the tablet wifi is enable
		self.pepper.tablet_service.enableWifi()
		
		#self.pepper.autonomous_life_off()
		#self.pepper.motion_service.wakeUp()
		#time.sleep(5)

		self.pepper.posture_service.goToPosture("StandInit",0.5)				
		#self.pepper.remoteTerminal()
		try:
			self.navigateToPoint()		
			self.pepper.tablet_service.hideImage()
			#self.videoStream()
			raw_input("\nSpeak to the robot using rules from both the activated topics. Press Enter when finished:")
		finally:
			print 'ok'

	if self.val == 3:
		self.arrive = 1
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

