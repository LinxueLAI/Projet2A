from robotDetector import *
from PIL import Image
from threading import Thread
import sys
import almath
import motion
import random
from numpy import array
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
	self.nbHuman = 0
	self.nbMasque = 0
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
	self.arrive == 1
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
                		self.pepper.getBehaviors(self.pepper.behavior_mng_service)
                		self.pepper.launchAndStopBehavior(self.pepper.behavior_mng_service, "dialogmask-9c9568/behavior_1")
				print "image output to the path: ./tmp/"+self.label+str(n)+".png"
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
		self.arrive == 0

    def detection(self):
	self.pepper.motion_service.setAngles("HeadPitch", -0.1, 0.2)
	self.arrive == 1
	tmp =  1
        theta = 0.5
        time_start = time.time()
        time_used = 0
        while self.arrive == 1 and time_used<15:
            	if tmp%3 == 1:
                	theta = 0.5
			print "left"
            	elif tmp%3 == 2:
                	theta = -0.5
			print "right"
            	else:
                	theta = 0
			print "mid"
            	# wait = np.random.randint(500, 1500) / 1000
            	# self.turn_around(theta)
            	self.pepper.motion_service.setAngles("HeadYaw", theta, 0.2)
            	time.sleep(2)
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
				self.arrive == 0
				print "arrived = "+str(self.arrive)
				self.pepper.blink_eyes([255, 0, 0])
				#self.pepper.tablet_service.showImage("https://png.pngtree.com/png-clipart/20200401/original/pngtree-hand-drawn-2019-new-corona-virus-wearing-a-mask-figure-png-image_5329325.jpg")
				#s = self.pepper.tablet_service.showImage("/home/nao/video/masque.jpg")
				if (self.pepper.behavior_mng_service.isBehaviorInstalled("ttablette-cf4635/behavior_1")):
				    # Check that it is not already running.
				    if (not self.pepper.behavior_mng_service.isBehaviorRunning("ttablette-cf4635/behavior_1")):
				    	self.pepper.behavior_mng_service.runBehavior("ttablette-cf4635/behavior_1",_async=True)
				    else:
				    	print "Behavior is already running.So we relaunch it."
					self.pepper.behavior_mng_service.stopBehavior("ttablette-cf4635/behavior_1")
			    		time.sleep(1.0)
					self.pepper.behavior_mng_service.runBehavior("ttablette-cf4635/behavior_1")
			    	else:
					print "Behavior not found."
				#print "s = "+str(s)
				#self.pepper.tablet_service.hideImage()
				self.nbHuman = self.nbHuman+1
                		self.pepper.getBehaviors(self.pepper.behavior_mng_service)
                		self.pepper.launchAndStopBehavior(self.pepper.behavior_mng_service, "dialogmask-9c9568/behavior_1")
				print "Behavior finished!"
				print "image output to the path: ./tmp/"+self.label+str(n)+".png"
				cv2.imwrite("./tmp/"+self.label+str(n)+".png", frame)
				print"I've met "+str(self.nbHuman)+" peoples,there're "+str(self.nbMasque)+" masks"
				self.pepper.tablet_service.hideImage()
				time.sleep(2)
			elif self.label=="Mask":
				self.arrive == 0		
				self.pepper.blink_eyes([0, 255, 0])
				self.nbHuman = self.nbHuman+1
				self.nbMasque = self.nbMasque+1
				self.pepper.say("C'est bien, vous portez votre masque.")
				print "image output"
				cv2.imwrite("./tmp/"+self.label+str(n)+".png", frame)
				print"I've met "+str(self.nbHuman)+" peoples,there're "+str(self.nbMasque)+" masks"		
				time.sleep(2)
			self.pepper.blink_eyes([0, 0, 255])
			self.label=""
			key = ""
		tmp = tmp+1
		time_used = time.time()-time_start
		if time_used>15:
			print "time's up!"
            	print "time used = "+str(time_used)
		
    def navigateToPoint(self):
        self.pepper.navigation_service.stopLocalization()
	#self.path="/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
	#self.path = "/home/nao/.local/share/Explorer/2015-09-18T210133.604Z.explo"
	self.path = "/home/nao/.local/share/Explorer/2015-09-28T210020.730Z.explo"
        self.pepper.navigation_service.loadExploration(str(self.path))

        self.pepper.navigation_service.startLocalization()
	self.time_used =0
        # time.sleep(5)
	self.time_start = time.time()
        self.home = self.pepper.navigation_service.getRobotPositionInMap()
        print "saved home position:"+str(self.home[0])
	p=array([[0.0,0.0],[2.0,0.0],[4.0,0.0],[6.0,0.0],[8.0,0.0],[10.0,0.0],[8.0,0.0],[6.0,0.0],[4.0,0.0],[2.0,0.0]])#right -   ;left +

        i = 0
        self.pepper.motion_service.setAngles("HeadPitch", -0.1, 0.2)
        while i<10:
            print "i = "+str(i)
            initPosition = almath.Pose2D(self.pepper.motion_service.getRobotPosition(True))
            print "initPos = "+str(initPosition)
            targetDistance = almath.Pose2D(int(p[i][0]),int(p[i][1]),0.0)
            expectedEndPosition = initPosition * targetDistance
            # pepper.motion_service.moveTo(3.0,0.0,0.0)
            print "go to point"+str(p[i][0])+","+str(p[i][1])
            self.pepper.navigation_service.navigateToInMap([int(p[i][0]),int(p[i][1]),0.0])
	    self.detection()
            print "position in map = "+str(self.pepper.navigation_service.getRobotPositionInMap()[0])
            realEndPosition = almath.Pose2D(self.pepper.motion_service.getRobotPosition(False))
            positionError = realEndPosition.diff(expectedEndPosition)
            print "positionError= "+str(positionError)
            positionError.theta = almath.modulo2PI(positionError.theta)

            if(positionError.x>0.3) or (positionError.y>0.3):
                print "continuer atteindre le point "+str(p[i][0])+","+str(p[i][1])
                self.pepper.navigation_service.navigateToInMap([int(p[i][0]),int(p[i][1]),0.0])
                print "position in map = "+str(self.pepper.navigation_service.getRobotPositionInMap()[0])

            print "now position = "+str(almath.Pose2D(self.pepper.motion_service.getRobotPosition(False)))
            print "arrive at "+str(p[i][0])+","+str(p[i][1])
            i = i + 1

	self.time_used = time.time()-self.time_start
	self.pepper.say("J'ai fini ma demonstration dans "+str(round(self.time_used,3))+" seconds. J'ai rencontrer "+str(self.nbHuman)+"personnes. Il y a "+str(self.nbMasque)+" masques.")

    def run(self):
	if self.val == 1:
		# Ensure that the tablet wifi is enable
		self.pepper.tablet_service.enableWifi()
		self.pepper.tablet_service.hideImage()
		#self.pepper.autonomous_life_off()
		#self.pepper.motion_service.wakeUp()
		#time.sleep(5)

		self.pepper.posture_service.goToPosture("StandInit",0.5)				
		#self.pepper.remoteTerminal()
		try:
			self.navigateToPoint()	
			# self.pepper.tablet_service.hideImage()
			#self.videoStream()
			raw_input("\n Press Enter when finished:")
		finally:
			print 'ok'

	if self.val == 2:
		#self.arrive = 1
		#self.parole()
		self.videoStream()

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

