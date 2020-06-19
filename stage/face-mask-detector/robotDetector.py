from robot_detect_image import *
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.video import VideoStream
import numpy as np
import argparse
import os
import qi
import time
import cv2
import speech_recognition
import gtts
import playsound
import subprocess
import socket
import imutils

class Pepper:
    def __init__(self, ip_address, port=9559):
        self.session = qi.Session()
        self.session.connect("tcp://" + ip_address + ":" + str(port))
        self.ip_address = ip_address
        self.port = port
        self.posture_service = self.session.service("ALRobotPosture")
        self.motion_service = self.session.service("ALMotion")
        self.tracker_service = self.session.service("ALTracker")
        self.tts_service = self.session.service("ALAnimatedSpeech")
        self.tablet_service = self.session.service("ALTabletService")
        self.autonomous_life_service = self.session.service("ALAutonomousLife")
        self.navigation_service = self.session.service("ALNavigation")
        self.awareness_service = self.session.service("ALBasicAwareness")
        self.led_service = self.session.service("ALLeds")
        self.camera_device = self.session.service("ALVideoDevice")
        self.face_detection_service = self.session.service("ALFaceDetection")
        self.memory_service = self.session.service("ALMemory")
        self.face_characteristic = self.session.service("ALFaceCharacteristics")
        self.people_perception = self.session.service("ALPeoplePerception")
        self.speech_service = self.session.service("ALSpeechRecognition")
        self.slam_map = None
        self.localization = None
        self.camera_link = None
	self.image = None
        print("[INFO]: Robot is initialized at " + ip_address + ":" + str(port))

    def unsubscribe_effector(self):
        """
        Unsubscribe a end-effector after tracking some object

        .. note:: It has to be done right after each tracking by hands.
        """
        self.tracker_service.unregisterAllTargets()
        self.tracker_service.setEffector("None")
        print("[INFO]: End-effector is unsubscribed")

    def subscribe_camera(self, camera, resolution, fps):
        color_space = 13
        camera_index = None
        if camera == "camera_top":
            camera_index = 0
        elif camera == "camera_bottom":
            camera_index = 1
        elif camera == "camera_depth":
            camera_index = 2
            resolution = 1
            color_space = 11
        self.camera_link = self.camera_device.subscribeCamera("Camera_Stream" + str(np.random.random()),
                                                              camera_index, resolution, color_space, fps)
        if self.camera_link:
            print("[INFO]: Camera is initialized")
        else:
            print("[ERROR]: Camera is not initialized properly")

    def say(self, text):
        self.tts_service.say(text)
        print("[INFO]: Robot says: " + text)

    def get_camera_frame(self, show):
        image_raw = self.camera_device.getImageRemote(self.camera_link)
        image = np.frombuffer(image_raw[6], np.uint8).reshape(image_raw[1], image_raw[0], 3)
        if show:
            cv2.imshow("Pepper Camera", image)
            cv2.waitKey(-1)
            cv2.destroyAllWindows()
        return image

    def stand(self):
        self.posture_service.goToPosture("Stand", 0.5)
        print("[INFO]: Robot is in default position")

    def unsubscribe_camera(self):
        self.camera_device.unsubscribe(self.camera_link)
        print("[INFO]: Camera was unsubscribed")

    def trackFace(self):
        face_found = False
        self.unsubscribe_effector()
	self.subscribe_camera("camera_top", 2, 30)
        self.posture_service.goToPosture("Stand", 0.5)
        self.say("Je cherche un visage.")
        proxy_name = "FaceDetection" + str(np.random)
	label=""
        print("[INFO]:looking for a face.")
	flow = True
	show = False
        while not face_found and flow==True:
	    if cv2.waitKey(1) & 0xFF == ord('q'):
		break
            self.face_detection_service.subscribe(proxy_name, 500, 0.0)
            for memory in range(5):
                time.sleep(0.5)
                output = self.memory_service.getData("FaceDetected")
                print("...")
                if output and isinstance(output, list) and len(output) >= 2:
                    print("Face detected")
		    print("timestamp=")
		    print(output[0])
		    print("faceinfoArray=")
		    print(output[1])
                    #print("FaceShapInfo=")
		    faceInfoArray=output[1]
		    for j in range( len(faceInfoArray)-1 ):
                	faceInfo = faceInfoArray[j]
                	# First Field = Shape info.
                	faceShapeInfo = faceInfo[0]
                	# Second Field = Extra info (empty for now).
                	faceExtraInfo = faceInfo[1]
                	print "Face Infos :  alpha %.3f - beta %.3f" % (faceShapeInfo[1], faceShapeInfo[2])
                	print "Face Infos :  width %.3f - height %.3f" % (faceShapeInfo[3], faceShapeInfo[4])
                	print "Face Extra Infos :" + str(faceExtraInfo)
			self.say("Salut,"+faceExtraInfo[2])
                    face_found = True
	    	    image = self.get_camera_frame(show)#true?
		    label = self.detect_image(image)
		    break

        self.tracker_service.registerTarget("Face", 0.15)
        self.tracker_service.setMode("Move")
        self.tracker_service.track("Face")
	#self.say("Veuillez attendre si vous plait.")
	boucle = 1
	while not label=="Mask":
		print "boucle="+str(boucle)
		text="Portez le masque si vous plait."
		if boucle == 1:		
			self.say(text)
		#time.sleep(2)
		image = self.get_camera_frame(show)#true?
		label = detect_image(image)
		boucle=boucle+1

	self.say("C'est bien, vous avez porter le masque.")
	self.unsubscribe_effector()
        self.stand()
       	self.face_detection_service.unsubscribe(proxy_name)	
	self.unsubscribe_camera()
	cv2.destroyAllWindows()

    def autonomous_life_off(self):
        self.autonomous_life_service.setState("disabled")
        self.stand()
        print("[INFO]: Autonomous life is off")

    def detect_image(self,image):
	# load our serialized face detector model from disk
	print("[INFO] loading face detector model...")
	prototxtPath = os.path.sep.join(["face_detector", "deploy.prototxt"])
	weightsPath = os.path.sep.join(["face_detector",
		"res10_300x300_ssd_iter_140000.caffemodel"])
	net = cv2.dnn.readNet(prototxtPath, weightsPath)

	# load the face mask detector model from disk
	print("[INFO] loading face mask detector model...")
	model = load_model("mask_detector.model")  # need tensorflow

	# load the input image from disk, clone it, and grab the image spatial
	# dimensions
	orig = image.copy()
	(h, w) = image.shape[:2]

	# construct a blob from the image
	blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300),
		(104.0, 177.0, 123.0))

	# pass the blob through the network and obtain the face detections
	print("[INFO] computing face detections...")
	net.setInput(blob)
	detections = net.forward()
	# loop over the detections
	for i in range(0, detections.shape[2]):
		# extract the confidence (i.e., probability) associated with
		# the detection
		confidence = detections[0, 0, i, 2]

		# filter out weak detections by ensuring the confidence is
		# greater than the minimum confidence
		if confidence > 0.5:
			# compute the (x, y)-coordinates of the bounding box for
			# the object
			box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
			(startX, startY, endX, endY) = box.astype("int")

			# ensure the bounding boxes fall within the dimensions of
			# the frame
			(startX, startY) = (max(0, startX), max(0, startY))
			(endX, endY) = (min(w - 1, endX), min(h - 1, endY))

			# extract the face ROI, convert it from BGR to RGB channel
			# ordering, resize it to 224x224, and preprocess it
			face = image[startY:endY, startX:endX]
			face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
			face = cv2.resize(face, (224, 224))
			face = img_to_array(face)       ## need tensorflow
			face = preprocess_input(face)   ## need tensorflow
			face = np.expand_dims(face, axis=0)

			# pass the face through the model to determine if the face
			# has a mask or not
			(mask, withoutMask) = model.predict(face)[0]

			# determine the class label and color we'll use to draw
			# the bounding box and text
			label = "Mask" if mask > withoutMask else "No Mask"
			color = (0, 255, 0) if label == "Mask" else (0, 0, 255)

			# include the probability in the label
			label1 = "{}: {:.2f}%".format(label, max(mask, withoutMask) * 100)

			# display the label and bounding box rectangle on the output
			# frame
			cv2.putText(image, label1, (startX, startY - 10),
				cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
			cv2.rectangle(image, (startX, startY), (endX, endY), color, 2)

	# show the output image
	#cv2.imshow("Output", image)
	cv2.imwrite("./tmp/"+label+".png", image)# save the image
	#cv2.waitKey(0)
	return label

    def detect_and_predict_mask(self,frame, faceNet, maskNet):
	# grab the dimensions of the frame and then construct a blob
	# from it
	(h, w) = frame.shape[:2]
	blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300),
		(104.0, 177.0, 123.0))

	# pass the blob through the network and obtain the face detections
	faceNet.setInput(blob)
	detections = faceNet.forward()

	# initialize our list of faces, their corresponding locations,
	# and the list of predictions from our face mask network
	faces = []
	locs = []
	preds = []

	# loop over the detections
	for i in range(0, detections.shape[2]):
		# extract the confidence (i.e., probability) associated with
		# the detection
		confidence = detections[0, 0, i, 2]

		# filter out weak detections by ensuring the confidence is
		# greater than the minimum confidence
		if confidence > 0.5:
			# compute the (x, y)-coordinates of the bounding box for
			# the object
			box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
			(startX, startY, endX, endY) = box.astype("int")

			# ensure the bounding boxes fall within the dimensions of
			# the frame
			(startX, startY) = (max(0, startX), max(0, startY))
			(endX, endY) = (min(w - 1, endX), min(h - 1, endY))

			# extract the face ROI, convert it from BGR to RGB channel
			# ordering, resize it to 224x224, and preprocess it
			face = frame[startY:endY, startX:endX]
			face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
			face = cv2.resize(face, (224, 224))
			face = img_to_array(face)
			face = preprocess_input(face)
			face = np.expand_dims(face, axis=0)

			# add the face and bounding boxes to their respective
			# lists
			faces.append(face)
			locs.append((startX, startY, endX, endY))

	# only make a predictions if at least one face was detected
	if len(faces) > 0:
		# for faster inference we'll make batch predictions on *all*
		# faces at the same time rather than one-by-one predictions
		# in the above `for` loop
		preds = maskNet.predict(faces)

	# return a 2-tuple of the face locations and their corresponding
	# locations
	return (locs, preds)
