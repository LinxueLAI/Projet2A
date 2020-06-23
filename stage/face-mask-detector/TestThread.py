from robotDetector import *
from PIL import Image
from threading import Thread
class Fios(Thread):
    def __init__(self,val):
	Thread.__init__(self)
	self.val=val
	self.pepper = Pepper("192.168.2.169", 9559)
	self.prototxtPath = os.path.sep.join(["face_detector", "deploy.prototxt"])
	self.weightsPath = os.path.sep.join(["face_detector","res10_300x300_ssd_iter_140000.caffemodel"])
	self.faceNet = cv2.dnn.readNet(self.prototxtPath, self.weightsPath)
	self.label = ""
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
	while True:
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

		if self.label=="No Mask":
			self.pepper.say("Portez le masque si vous plait.")
			print "image output"
			cv2.imwrite("./tmp/"+self.label+".png", frame)
			time.sleep(2)
		elif self.label=="Mask":
			self.pepper.say("C'est bien, vous avez porter le masque.")
			print "image output"
			cv2.imwrite("./tmp/"+self.label+".png", frame)		
			time.sleep(2)
		self.label=""
		key = ""
		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			break

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
        #loads the map from the directory where it was saved during exploration
        self.pepper.navigation_service.loadExploration(str(path))
	guess =[0.,0.]
        self.pepper.navigation_service.relocalizeInMap(guess)
        self.pepper.navigation_service.startLocalization()
        
        #the map made by the robot has associated a coordinate system relative to the robot. the place where the robot starts the movement is the [0.0,0.0,0.0]. we define this position as our home
        self.home = self.pepper.navigation_service.getRobotPositionInMap()
        print "saved home position"
        
	#self.pepper.say("Je peux marcher.")
        #self.pepper.motion_service.moveTo(3.0,0.0,0.0)
        self.pepper.navigation_service.navigateToInMap([3.,0.,0.])
        self.a = self.pepper.navigation_service.getRobotPositionInMap() #a is defined in the same position as home. the reason for this is to follow a logic when it is called in the steps.
        a=self.a 
        print "point a ="+str(a)
        
        result_map = self.pepper.navigation_service.getMetricalMap()
        map_width = result_map[1]
        map_height = result_map[2]
        img = np.array(result_map[4]).reshape(map_width, map_height)
        img = (100 - img) * 2.55 # from 0..100 to 255..0
        img = np.array(img, np.uint8)
        Image.frombuffer('L',  (map_width, map_height), img, 'raw', 'L', 0, 1).show()

    def run(self):
	if self.val == 1:
		#self.videoStream()
		self.def_point_show()
		#self.steps_show()
	#if self.val == 2:
		#self.parole()

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

