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
from PIL import Image
import sys
import almath
import motion
import random
# ==============================================================================
#                      --- CAMERA INFORMATION ---

# AL_resolution
AL_kQQQQVGA = 8 	#Image of 40*30px
AL_kQQQVGA  = 7 	#Image of 80*60px
AL_kQQVGA   = 0 	#Image of 160*120px
AL_kQVGA    = 1 	#Image of 320*240px
AL_kVGA     = 2 	#Image of 640*480px
AL_k4VGA    = 3 	#Image of 1280*960px
AL_k16VGA   = 4 	#Image of 2560*1920px

# Camera IDs
AL_kTopCamera    = 0
AL_kBottomCamera = 1
AL_kDepthCamera  = 2

# Need to add All color space variables
AL_kBGRColorSpace = 13

# ==============================================================================
class Pepper:
    def __init__(self, ip_address, port=9559):
        self.session = qi.Session()
        self.session.connect("tcp://" + ip_address + ":" + str(port))
        self.ip_address = ip_address
        self.port = port
        # SUBSCRIBING SERVICES
        self.posture_service = self.session.service("ALRobotPosture")
        self.motion_service = self.session.service("ALMotion")
        self.tracker_service = self.session.service("ALTracker")
        self.tts_service = self.session.service("ALTextToSpeech")#ALAnimatedSpeech/ALTextToSpeech
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
        self.animation_player_service = self.session.service("ALAnimationPlayer")
        self.dialog_service           = self.session.service("ALDialog")
        # INITIALISING CAMERA POINTERS
        self.imageNo2d                = 1
        self.imageNo3d                = 1

        self.slam_map = None
        self.localization = None
        self.camera_link = None
	self.image = None
        stiffnesses  = 1.0
        names ="Body"
        self.motion_service.setStiffnesses(names, stiffnesses)
	self.motion_service.setOrthogonalSecurityDistance(0.05)
        print("[INFO]: Robot is initialized at " + ip_address + ":" + str(port))

    def _userArmArticular(self):
        # If needed to make hands move while moving.
        # Arms motion from user have always the priority than walk arms motion

        JointNames = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
        Arm1 = [-40,  25, 0, -40]
        Arm1 = [ x * motion.TO_RAD for x in Arm1]

        Arm2 = [-40,  50, 0, -80]
        Arm2 = [ x * motion.TO_RAD for x in Arm2]

        pFractionMaxSpeed = 0.6

        self.motion_service.angleInterpolationWithSpeed(JointNames, Arm1, pFractionMaxSpeed)
        self.motion_service.angleInterpolationWithSpeed(JointNames, Arm2, pFractionMaxSpeed)
        self.motion_service.angleInterpolationWithSpeed(JointNames, Arm1, pFractionMaxSpeed)

        return

    def _rotateHead(self):

        JointNamesH = ["HeadPitch", "HeadYaw"] # range ([-1,1],[-0.5,0.5]) // HeadPitch :{(-)up,(+)down} , HeadYaw :{(-)left,(+)right}

        amntY =  raw_input("Enter amount to Move Up(-) And Down(+) [-1,1] : ")
        amntX =  raw_input("Enter amount to Move Left(-) And Right(+) [-0.5,0.5] : ")

        pFractionMaxSpeed = 0.2

        HeadA = [float(amntY),float(amntX)]

        self.motion_service.angleInterpolationWithSpeed(JointNamesH, HeadA, pFractionMaxSpeed)
        time.sleep(1)

        return

    def _capture2dImage(self, cameraId):
        # Capture Image in RGB

        # WARNING : The same Name could be used only six time.
        strName = "capture2DImage_{}".format(random.randint(1,10000000000))
        clientRGB = self.camera_device.subscribeCamera(strName, cameraId, AL_kVGA, 11, 10)
        imageRGB = self.camera_device.getImageRemote(clientRGB)

        imageWidth = imageRGB[0]
        imageHeight = imageRGB[1]
        array = imageRGB[6]
        image_string = str(bytearray(array))

        # Create a PIL Image from our pixel array.
        im = Image.frombytes("RGB", (imageWidth, imageHeight), image_string)

        # Save the image inside the images foler in pwd.
        image_name_2d = "tmp/img2d-" + str(self.imageNo2d) + ".png"
        im.save(image_name_2d, "PNG") # Stored in images folder in the pwd, if not present then create one
        self.imageNo2d += 1
        im.show()

        return

    def _capture3dImage(self):
        # Depth Image in RGB

        # WARNING : The same Name could be used only six time.
        strName = "capture3dImage_{}".format(random.randint(1,10000000000))

        clientRGB = self.camera_device.subscribeCamera(strName, AL_kDepthCamera, AL_kQVGA, 11, 15)
        imageRGB = self.camera_device.getImageRemote(clientRGB)

        imageWidth = imageRGB[0]
        imageHeight = imageRGB[1]
        array = imageRGB[6]
        image_string = str(bytearray(array))

        # Create a PIL Image from our pixel array.
        im = Image.frombytes("RGB", (imageWidth, imageHeight), image_string)

        # Save the image inside the images foler in pwd.
        image_name_3d = "tmp/img3d-" + str(self.imageNo3d) + ".png"
        im.save(image_name_3d, "PNG") # Stored in images folder in the pwd, if not present then create one
        self.imageNo3d += 1
        im.show()

        return

    '''
    MOVERMENTS ARE RELATIVE HERE NOT ABSOLUTE, FOR ABSOLUTE MOVEMENT ONE CAN
    USE ANGLES AND USE 'moveTo()' METHOD FOR IT.
    '''
    def _moveForward(self, amnt):
        #TARGET VELOCITY
        X = 0.5  # forward
        Y = 0.0
        Theta = 0.0

        try:
            self.motion_service.moveToward(X, Y, Theta)
        except KeyboardInterrupt:
            print "KeyBoard Interrupt initiated"
            self.motion_service.stopMove()
            exit()
        except Exception, errorMsg:
            print str(errorMsg)
            print "This example is not allowed on this robot."
            exit()

        print "====================================================================="
        print "Forward Movement Started"
        time.sleep(float(amnt))
        print "Forward Movement Complete"
        print "====================================================================="

        return

    def navigateToPoint(self,x,y):
        self.navigation_service.stopLocalization()
	self.path="/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
        self.navigation_service.loadExploration(str(self.path))

        self.navigation_service.startLocalization()
        print "go to point : "+str(x)+","+str(y)
        self.navigation_service.navigateToInMap([x,y,0.0])
	print "reached e:"+str(self.navigation_service.getRobotPositionInMap()[0])
        self.posture_service.goToPosture("StandInit",0.5)

    # @WARNING : Dont Use this as Pepper doesn't have a sensor at its back
    def _moveBackward(self, amnt):
        #TARGET VELOCITY
        X = -0.5  # backward
        Y = 0.0
        Theta = 0.0

        try:
            self.motion_service.moveToward(X, Y, Theta)
        except KeyboardInterrupt:
            print "KeyBoard Interrupt initiated"
            self.motion_service.stopMove()
            exit()
        except Exception, errorMsg:
            print str(errorMsg)
            print "This example is not allowed on this robot."
            exit()
        amnt = 1.0 # default overiding the original value , JUST FOR CAUTION
        print "====================================================================="
        print "Backward Movement Started"
        time.sleep(float(amnt))
        print "Backward Movement Complete"
        print "====================================================================="

        return

    def _rotateRight(self, amnt):
        #TARGET VELOCITY
        X = 0.0
        Y = 0.0
        Theta = -0.5

        try:
            self.motion_service.moveToward(X, Y, Theta)
        except KeyboardInterrupt:
            print "KeyBoard Interrupt initiated"
            self.motion_service.stopMove()
            exit()
        except Exception, errorMsg:
            print str(errorMsg)
            print "This example is not allowed on this robot."
            exit()

        print "====================================================================="
        print "Rotate Right Movement Started"
        time.sleep(float(amnt))
        print "Rotate Right Movement Complete"
        print "====================================================================="

        return

    def _rotateLeft(self, amnt):
        #TARGET VELOCITY
        X = 0.0
        Y = 0.0
        Theta = 0.5

        try:
            self.motion_service.moveToward(X, Y, Theta)
        except KeyboardInterrupt:
            print "KeyBoard Interrupt initiated"
            self.motion_service.stopMove()
            exit()
        except Exception, errorMsg:
            print str(errorMsg)
            print "This example is not allowed on this robot."
            exit()

        print "====================================================================="
        print "Rotate Left Movement Started"
        time.sleep(float(amnt))
        print "Rotate Left Movement Complete"
        print "====================================================================="
        return

    def remoteTerminal(self):
        print " === START === "
        # Wake up robot
        # self.autonomous_life_off()
        self.motion_service.wakeUp()
        # Send robot to Stand
        self.posture_service.goToPosture("StandInit", 0.5)
        #####################
        ## Enable arms control by Motion algorithm
        #####################
        self.motion_service.setMoveArmsEnabled(True, True)
        #####################
        ## FOOT CONTACT PROTECTION
        #####################
        self.motion_service.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
        while(1):
            try:
                in_command = raw_input("Enter Command : ")
                if in_command == "w" : # moveForward
                    try:
                        amnt = raw_input("Enter Value : ")
                        self._moveForward(amnt)
                        self.motion_service.stopMove()
                    except KeyboardInterrupt:
                        print "KeyBoard Interrupt initiated"
                        self.motion_service.stopMove()
                        exit()
                    except Exception, errorMsg:
                        print str(errorMsg)
                        print "This example is not allowed on this robot."
                        exit()
                if in_command == "back" : # moveBackward
                    try:
                        amnt = raw_input("Enter Value : ")
                        self._moveBackward(amnt)
                        self.motion_service.stopMove()
                    except KeyboardInterrupt:
                        print "KeyBoard Interrupt initiated"
                        self.motion_service.stopMove()
                        exit()
                    except Exception, errorMsg:
                        print str(errorMsg)
                        print "This example is not allowed on this robot."
                        exit()
                if in_command == "a" : # rotateLeft
                    try:
                        amnt = raw_input("Enter Value : ")
                        self._rotateLeft(amnt)
                        self.motion_service.stopMove()
                    except KeyboardInterrupt:
                        print "KeyBoard Interrupt initiated"
                        self.motion_service.stopMove()
                        exit()
                    except Exception, errorMsg:
                        print str(errorMsg)
                        print "This example is not allowed on this robot."
                        exit()
                if in_command == "d" : # rotateRight
                    try:
                        amnt = raw_input("Enter Value : ")
                        self._rotateRight(amnt)
                        self.motion_service.stopMove()
                    except KeyboardInterrupt:
                        print "KeyBoard Interrupt initiated"
                        self.motion_service.stopMove()
                        exit()
                    except Exception, errorMsg:
                        print str(errorMsg)
                        print "This example is not allowed on this robot."
                        exit()
                if in_command == "s" : # speak
                    try:
                        language = raw_input("Enter language To Speak : ")
                        self.dialog_service.setLanguage(language)
                        user_msg = raw_input("Enter Sentence To Speak : ")
                        future = self.animation_player_service.run("animations/Stand/Gestures/Give_3", _async=True)
                        sentence = "\RSPD="+ str( 100 ) + "\ "
                        sentence += "\VCT="+ str( 100 ) + "\ "
                        sentence += user_msg
                        sentence +=  "\RST\ "
                        self.tts_service.say(str(sentence))
                        future.value()
                        self.posture_service.goToPosture("StandInit", 0.5)

                    except KeyboardInterrupt:
                        print "KeyBoard Interrupt initiated"
                        self.motion_service.stopMove()
                        self.posture_service.goToPosture("StandInit", 0.5)
                        future.cancel()
                        exit()
                    except Exception, errorMsg:
                        print str(errorMsg)
                        print "This example is not allowed on this robot."
                        exit()

                if in_command == "c2d" : # capture2dImage
                    try:
                        cameraId = raw_input("Enter CameraId [TopCamera : 0, BottomCamera : 1] : ")
                        self._capture2dImage(int(cameraId))
                    except KeyboardInterrupt:
                        print "KeyBoard Interrupt initiated"
                        exit()
                    except Exception, errorMsg:
                        print str(errorMsg)
                        print "This example is not allowed on this robot."
                        exit()

                if in_command == "c3d" : # capture3dImage
                    try:
                        self._capture3dImage()
                    except KeyboardInterrupt:
                        print "KeyBoard Interrupt initiated"
                        exit()
                    except Exception, errorMsg:
                        print str(errorMsg)
                        print "This example is not allowed on this robot."
                        exit()

                if in_command == "h" : # rotateHead
                    try:
                        self._rotateHead()
                        self.motion_service.stopMove()
                    except KeyboardInterrupt:
                        print "KeyBoard Interrupt initiated"
                        self.motion_service.stopMove()
                        exit()
                    except Exception, errorMsg:
                        print str(errorMsg)
                        print "This example is not allowed on this robot."
                        exit()

                if in_command == "to" : # navigationTo
                    try:
			positionX = raw_input("Enter positionX : ")
                        positionY = raw_input("Enter positionY : ")
                        self.navigateToPoint(int(positionX),int(positionY))
                        self.motion_service.stopMove()
                    except KeyboardInterrupt:
                        print "KeyBoard Interrupt initiated"
                        self.motion_service.stopMove()
                        exit()
                    except Exception, errorMsg:
                        print str(errorMsg)
                        print "This example is not allowed on this robot."
                        exit()

                if in_command == "x" : # exiting
                    print "Exiting gracefully"
                    break;

            except Exception, errorMsg:
                print str(errorMsg)
                print "This example is not allowed on this robot."
                exit()
        print "====================================================================="
        print "End Movement"
        print "====================================================================="
        # Go to rest position
        #self.motion_service.rest()
        return

    def explore(self, radius, force=False):
        self.motion_service.wakeUp() #it is only used if the robot is in rest position.
        # Explore the environement
        # radius = 3.0
        error = self.navigation_service.explore(radius)
        if error != 0:
            print "Exploration failed."
            return
        # Saves the exploration on the robots disk
        self.path = self.navigation_service.saveExploration()
        print "Exploration saved at path: \"" + self.path + "\""
        # Start localization to navigate in map
        print "now the robot: startLocalisation "
        self.navigation_service.startLocalization()
        # Come back to initial position
        print "now the robot: it begins to go to initial position "
        self.navigation_service.navigateToInMap([0., 0., 0.])
        # Stop localization
        print "now the robot: it has arrived the initial position "
        self.navigation_service.stopLocalization()
        # Retrieve and display the map built by the robot
        result_map = self.navigation_service.getMetricalMap()
	print"resolution="+str(result_map[0])
        map_width = result_map[1]
        map_height = result_map[2]
        img = np.array(result_map[4]).reshape(map_width, map_height)
        img = (100 - img) * 2.55 # from 0..100 to 255..0
        img = np.array(img, np.uint8)
        Image.frombuffer('L',  (map_width, map_height), img, 'raw', 'L', 0, 1).show()
        self.posture_service.goToPosture("StandInit",0.5)

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

    def listen_to(self, vocabulary):
        """
        Listen and match the vocabulary which is passed as parameter.

        :Example:

        >>> words = pepper.listen_to(["what color is the sky", "yes", "no"]

        :param vocabulary: List of phrases or words to recognize
        :type vocabulary: string
        :return: Recognized phrase or words
        :rtype: string
        """
        self.speech_service.setLanguage("French")#English
        self.speech_service.pause(True)
        try:
            self.speech_service.setVocabulary(vocabulary, True)
        except RuntimeError as error:
            print(error)
            self.speech_service.removeAllContext()
            self.speech_service.setVocabulary(vocabulary, True)
            self.speech_service.subscribe("Test_ASR")
        try:
            print("[INFO]: Robot is listening to you...")
            self.speech_service.pause(False)
            time.sleep(4)
            words = self.memory_service.getData("WordRecognized")
            print("[INFO]: Robot understood: '" + words[0] + "'")
            return words[0]
        except:
            pass

    def listen(self):
        """
        Wildcard speech recognition via internal Pepper engine

        .. warning:: To get this proper working it is needed to disable or uninstall \
        all application which can modify a vocabulary in a Pepper.

        .. note:: Note this version only rely on time but not its internal speak processing \
        this means that Pepper will 'bip' at the begining and the end of human speak \
        but it is not taken a sound in between the beeps. Search for 'Robot is listening to \
        you ... sentence in log console

        :Example:

        >>> words = pepper.listen()

        :return: Speech to text
        :rtype: string
        """
        self.speech_service.setAudioExpression(False)
        self.speech_service.setVisualExpression(False)
        self.audio_recorder.stopMicrophonesRecording()
        print("[INFO]: Speech recognition is in progress. Say something.")
        while True:
            print(self.memory_service.getData("ALSpeechRecognition/Status"))
            if self.memory_service.getData("ALSpeechRecognition/Status") == "SpeechDetected":
                self.audio_recorder.startMicrophonesRecording("/home/nao/speech.wav", "wav", 48000, (0, 0, 1, 0))
                print("[INFO]: Robot is listening to you")
                self.blink_eyes([255, 255, 0])
                break

        while True:
            if self.memory_service.getData("ALSpeechRecognition/Status") == "EndOfProcess":
                self.audio_recorder.stopMicrophonesRecording()
                print("[INFO]: Robot is not listening to you")
                self.blink_eyes([0, 0, 0])
                break

        self.download_file("speech.wav")
        self.speech_service.setAudioExpression(True)
        self.speech_service.setVisualExpression(True)

        return self.speech_to_text("speech.wav")

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

    def blink_eyes(self, rgb):
        """
        Blink eyes with defined color

        :param rgb: Color in RGB space
        :type rgb: integer

        :Example:

        >>> pepper.blink_eyes([255, 0, 0])

        """
        self.led_service.fadeRGB('AllLeds', rgb[0], rgb[1], rgb[2], 1.0)

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

    def autonomous_life_on(self):
        self.autonomous_life_service.setState("interactive")
        print("[INFO]: Autonomous life is on")

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


"""
    mouvement
"""

    def dance(self,motion_service):
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

        self.motion_service.angleInterpolation(names, keys, times, True)

    def wave(self,motion_service):
        # Choregraphe bezier export in Python.
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([0.1, 11.9])
        keys.append([-0.199418, -0.199418])

        names.append("HeadYaw")
        times.append([0.1, 11.9])
        keys.append([0, 0])

        names.append("HipPitch")
        times.append([0.1, 11.9])
        keys.append([-0.0322137, -0.0322137])

        names.append("HipRoll")
        times.append([0.1, 11.9])
        keys.append([-0.0107379, -0.0107379])

        names.append("KneePitch")
        times.append([0.1, 11.9])
        keys.append([0.0107379, 0.0107379])

        names.append("LElbowRoll")
        times.append([0.1, 0.9, 1.4, 1.9, 2.4, 2.9, 3.4, 3.9, 4.4, 4.9, 5.4, 5.9, 6.4, 6.9, 7.4, 7.9, 8.4, 8.9, 9.4, 9.9, 10.4, 10.9, 11.9])
        keys.append([-0.523088, -1.14319, -0.830777, -1.14319, -1.33867, -1.14319, -0.830777, -1.14319, -1.33867, -1.14319, -0.830777, -1.14319, -1.33867, -1.14319, -0.830777, -1.14319, -1.33867, -1.14319, -0.755728, -1.14319, -1.33867, -1.14319, -0.523087])

        names.append("LElbowYaw")
        times.append([0.1, 0.9, 1.4, 1.9, 2.4, 2.9, 3.4, 3.9, 4.4, 4.9, 5.4, 5.9, 6.4, 6.9, 7.4, 7.9, 8.4, 8.9, 9.4, 9.9, 10.4, 10.9, 11.9])
        keys.append([-1.23792, -1.39103, -2.08567, -1.39103, -0.825541, -1.39103, -2.08567, -1.39103, -0.825541, -1.39103, -2.08567, -1.39103, -0.825541, -1.39103, -2.08567, -1.39103, -0.825541, -1.39103, -2.08567, -1.39103, -0.825541, -1.39103, -1.23792])

        names.append("LHand")
        times.append([0.1, 0.9, 1.4, 1.9, 2.4, 2.9, 3.4, 3.9, 4.4, 4.9, 5.4, 5.9, 6.4, 6.9, 7.4, 7.9, 8.4, 8.9, 9.4, 9.9, 10.4, 10.9, 11.9])
        keys.append([0.589631, 0.87, 0.78, 0.87, 0.87, 0.87, 0.78, 0.87, 0.87, 0.87, 0.78, 0.87, 0.87, 0.87, 0.78, 0.87, 0.87, 0.87, 0.78, 0.87, 0.87, 0.87, 0.589631])

        names.append("LShoulderPitch")
        times.append([0.1, 0.9, 1.4, 1.9, 2.4, 2.9, 3.4, 3.9, 4.4, 4.9, 5.4, 5.9, 6.4, 6.9, 7.4, 7.9, 8.4, 8.9, 9.4, 9.9, 10.4, 10.9, 11.9])
        keys.append([1.56159, -0.246576, -0.1309, -0.249582, -0.249582, -0.249582, -0.1309, -0.249582, -0.249582, -0.249582, -0.1309, -0.249582, -0.249582, -0.249582, -0.1309, -0.249582, -0.249582, -0.249582, -0.1309, -0.249582, -0.249582, -0.249582, 1.56159])

        names.append("LShoulderRoll")
        times.append([0.1, 0.9, 1.4, 1.9, 2.4, 2.9, 3.4, 3.9, 4.4, 4.9, 5.4, 5.9, 6.4, 6.9, 7.4, 7.9, 8.4, 8.9, 9.4, 9.9, 10.4, 10.9, 11.9])
        keys.append([0.144194, 0.678972, 0.328122, 0.383972, 0.383972, 0.383972, 0.328122, 0.383972, 0.383972, 0.383972, 0.328122, 0.383972, 0.383972, 0.383972, 0.328122, 0.383972, 0.383972, 0.383972, 0.328122, 0.383972, 0.383972, 0.383972, 0.144194])

        names.append("LWristYaw")
        times.append([0.1, 0.9, 1.4, 1.9, 2.4, 2.9, 3.4, 3.9, 4.4, 4.9, 5.4, 5.9, 6.4, 6.9, 7.4, 7.9, 8.4, 8.9, 9.4, 9.9, 10.4, 10.9, 11.9])
        keys.append([0.0137641, 0.87441, 1.44339, 0.87441, 0.87441, 0.87441, 1.44339, 0.87441, 0.87441, 0.87441, 1.44339, 0.87441, 0.87441, 0.87441, 1.44339, 0.87441, 0.87441, 0.87441, 1.44339, 0.87441, 0.87441, 0.87441, 0.0137641])

        names.append("RElbowRoll")
        times.append([0.1, 11.9])
        keys.append([0.523088, 0.523087])

        names.append("RElbowYaw")
        times.append([0.1, 11.9])
        keys.append([1.23025, 1.23025])

        names.append("RHand")
        times.append([0.1, 11.9])
        keys.append([0.589631, 0.589631])

        names.append("RShoulderPitch")
        times.append([0.1, 11.9])
        keys.append([1.55852, 1.55852])

        names.append("RShoulderRoll")
        times.append([0.1, 11.9])
        keys.append([-0.14266, -0.14266])

        names.append("RWristYaw")
        times.append([0.1, 11.9])
        keys.append([0.00149202, 0.00149202])
        try:
        # uncomment the following line and modify the IP if you use this script outside Choregraphe.
        # motion = ALProxy("ALMotion", IP, 9559)
            self.motion_service.angleInterpolation(names, keys, times, True)
        except BaseException, err:
            print err
    def taijiquan(self,motion_service):
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([2.8, 4.8, 6.8, 8.8, 10.8, 12.8, 14.8, 16.8, 18.8, 20.8, 23.4, 26, 28.2, 30.2, 32.2, 34.2, 36.8, 39.4, 42, 44.2, 46, 49.8])
        keys.append([0, 8.95233e-08, -4.76838e-07, 8.89455e-08, 1.04976e-07, 0.331613, 0.314159, 9.19019e-08, -0.331613, 0.139626, -0.0872665, 0.139626, 0.383972, 0.558505, 0.383972, -0.331613, 0.139626, -0.0872665, 0.139626, 0.383972, 0, -0.190258])

        names.append("HeadYaw")
        times.append([2.8, 4.8, 6.8, 8.8, 10.8, 12.8, 14.8, 16.8, 18.8, 20.8, 23.4, 26, 28.2, 30.2, 32.2, 34.2, 36.8, 39.4, 42, 44.2, 46, 49.8])
        keys.append([0, 8.42936e-08, 8.42938e-08, 8.42938e-08, -4.76838e-07, 0.314159, -0.296706, -1.18682, -0.279253, 0.20944, 1.5708, 0.20944, 0.139626, 0, -0.139626, 0.279253, -0.20944, -1.5708, -0.20944, -0.139626, 0, -0.00310993])

        names.append("HipPitch")
        times.append([6.8, 10.8, 12.8, 18.8, 28.2, 34.2, 36.8, 39.4, 42, 44.2])
        keys.append([-0.307178, -0.0444883, -0.444757, -0.0454611, -0.444757, -0.0454611, -0.444757, -0.0454611, -0.444757, -0.0454611])

        names.append("HipRoll")
        times.append([10.8, 12.8, 18.8, 28.2, 34.2, 36.8, 39.4, 42, 44.2])
        keys.append([0.000864661, 0.00719934, 0.00719934, 0.00719934, 0.00719934, 0.00719934, 0.00719934, 0.00719934, 0.00719934])

        names.append("KneePitch")
        times.append([6.8, 10.8, 12.8, 18.8, 28.2, 34.2, 36.8, 39.4, 42, 44.2])
        keys.append([0.0994838, -0.0143759, 0.178257, -0.0113593, 0.178257, -0.0113593, 0.178257, -0.0113593, 0.178257, -0.0113593])

        names.append("LElbowRoll")
        times.append([2.8, 4.8, 6.8, 8.8, 10.8, 12.8, 14.8, 16.8, 18.8, 20.8, 23.4, 26, 28.2, 30.2, 32.2, 34.2, 36.8, 39.4, 42, 44.2, 45.2, 46, 49.8])
        keys.append([0, -0.698132, -1.0472, 0, 0, -1.65806, -0.959931, -1.48353, -1.01229, -1.01229, 0, -1.01229, -1.01229, -0.890118, -0.855211, -1.11701, -0.855211, -1.25664, -0.855211, -0.855211, -0.994838, -1.4207, -0.38806])

        names.append("LElbowYaw")
        times.append([2.8, 4.8, 6.8, 8.8, 10.8, 12.8, 14.8, 16.8, 18.8, 20.8, 23.4, 26, 28.2, 30.2, 32.2, 34.2, 36.8, 39.4, 42, 44.2, 45.2, 46, 49.8])
        keys.append([-1.5708, -1.5708, -1.5708, -1.5708, -1.5708, -0.383972, 0, 0, 0, 0, 0, 0, 0, 0.20944, 0.191986, -0.418879, -0.418879, -0.0872665, -0.418879, 0.191986, -0.378736, -0.244346, -1.18276])

        names.append("LHand")
        times.append([2.8, 49.8])
        keys.append([0, 0.2984])

        names.append("LShoulderPitch")
        times.append([2.8, 4.8, 6.8, 8.8, 10.8, 12.8, 14.8, 16.8, 18.8, 20.8, 23.4, 26, 28.2, 30.2, 32.2, 34.2, 36.8, 39.4, 42, 44.2, 46, 49.8])
        keys.append([1.5708, 1.91986, 2.0944, 1.5708, 0, 0.366519, 0.349066, 0.191986, -0.802851, -0.174533, -0.296706, -0.174533, 0.523599, 0.471239, 0.331613, -0.471239, 0.0698132, -0.0698132, 0.0698132, 0.331613, 1.69297, 1.52936])

        names.append("LShoulderRoll")
        times.append([2.8, 4.8, 6.8, 8.8, 10.8, 12.8, 14.8, 16.8, 18.8, 20.8, 23.4, 26, 28.2, 30.2, 32.2, 34.2, 36.8, 39.4, 42, 44.2, 46, 49.8])
        keys.append([0.174533, 0.349066, 0.174533, 0.174533, 0.174533, 0.698132, 0, 0.0872665, 0.174533, 0.401426, 1.15192, 0.401426, 0.401426, 0.174533, 0, 0.401426, 0, 0, 0, 0.20944, 0.942478, 0.107338])

        names.append("LWristYaw")
        times.append([2.8, 49.8])
        keys.append([-1.53589, 0.139552])

        names.append("RElbowRoll")
        times.append([2.8, 4.8, 6.8, 8.8, 10.8, 12.8, 14.8, 16.8, 18.8, 20.8, 23.4, 26, 28.2, 30.2, 32.2, 34.2, 36.8, 39.4, 42, 44.2, 45.2, 46, 49.8])
        keys.append([0, 0.698132, 1.0472, 2.57424e-07, 0, 1.23918, 1.64061, 0.0698132, 1.11701, 0.855211, 1.25664, 0.855211, 0.855211, 0.890118, 1.01229, 1.01229, 1.01229, 0.0349066, 1.01229, 1.01229, 1.13272, 1.36659, 0.395814])

        names.append("RElbowYaw")
        times.append([2.8, 4.8, 6.8, 8.8, 10.8, 12.8, 14.8, 16.8, 18.8, 20.8, 23.4, 26, 28.2, 30.2, 32.2, 34.2, 36.8, 39.4, 42, 44.2, 45.2, 46, 49.8])
        keys.append([1.5708, 1.5708, 1.5708, 1.5708, 1.5708, 0.191986, 0.349066, 1.5708, 0.418879, 0.418879, 0.0872665, 0.418879, -0.191986, -0.20944, 0, 0, 0, 0, 0, 0, 0.342085, 0.244346, 1.15966])

        names.append("RHand")
        times.append([2.8, 49.8])
        keys.append([0, 0.302])

        names.append("RShoulderPitch")
        times.append([2.8, 4.8, 6.8, 8.8, 10.8, 12.8, 14.8, 16.8, 18.8, 20.8, 23.4, 26, 28.2, 30.2, 32.2, 34.2, 36.8, 39.4, 42, 44.2, 46, 49.8])
        keys.append([1.5708, 1.91986, 2.0944, 1.5708, 0, 0.174533, 0.610865, 1.0472, -0.471239, 0.0698132, -0.0698132, 0.0698132, 0.331613, 0.471239, 0.523599, -0.802851, -0.174533, -0.296706, -0.174533, 0.523599, 1.69297, 1.51563])

        names.append("RShoulderRoll")
        times.append([2.8, 4.8, 6.8, 8.8, 10.8, 12.8, 14.8, 16.8, 18.8, 20.8, 23.4, 26, 28.2, 30.2, 32.2, 34.2, 36.8, 39.4, 42, 44.2, 46, 49.8])
        keys.append([-0.174533, -0.174533, -0.349066, -0.174533, -0.174515, -0.0698132, -0.837758, -1.51844, -0.401426, 0, 0, 0, 0, -0.174533, -0.401426, -0.174533, -0.401426, -1.15192, -0.401426, -0.558505, -0.942478, -0.099752])

        names.append("RWristYaw")
        times.append([2.8, 49.8])
        keys.append([1.53589, 0.164096])

        try:
        # uncomment the following line and modify the IP if you use this script outside Choregraphe.
        # motion = ALProxy("ALMotion", IP, 9559)
            self.motion_service.angleInterpolation(names, keys, times, True)
        except BaseException, err:
            print err


    def wavewithhand(self,motion_service):
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([0.1, 46.1])
        keys.append([-0.199418, -0.199418])

        names.append("HeadYaw")
        times.append([0.1, 46.1])
        keys.append([0, 0])

        names.append("HipPitch")
        times.append([0.1, 46.1])
        keys.append([-0.0322137, -0.0322137])

        names.append("HipRoll")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([-0.0107379, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.0107379])

        names.append("KneePitch")
        times.append([0.1, 46.1])
        keys.append([0.0107379, 0.0107379])

        names.append("LElbowRoll")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([-0.523087, -0.911062, -0.202458, -0.911062, -0.202458, -0.911062, -0.202458, -0.911062, -0.202458, -0.911062, -0.202458, -0.911062, -0.202458, -0.911062, -0.202458, -0.911062, -0.202458, -0.911062, -0.202458, -0.911062, -0.202458, -0.911062, -0.202458, -0.523087])

        names.append("LElbowYaw")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([-1.23792, -0.434587, -1.83085, -0.434587, -1.83085, -0.434587, -1.83085, -0.434587, -1.83085, -0.434587, -1.83085, -0.434587, -1.83085, -0.434587, -1.83085, -0.434587, -1.83085, -0.434587, -1.83085, -0.434587, -1.83085, -0.434587, -1.83085, -1.23792])

        names.append("LHand")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([0.589631, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.589631])

        names.append("LShoulderPitch")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([1.56159, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, 1.56159])

        names.append("LShoulderRoll")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([0.144194, 0.261799, 0.76969, 0.261799, 0.76969, 0.261799, 0.76969, 0.261799, 0.76969, 0.261799, 0.76969, 0.261799, 0.76969, 0.261799, 0.76969, 0.261799, 0.76969, 0.261799, 0.76969, 0.261799, 0.76969, 0.261799, 0.76969, 0.144194])

        names.append("LWristYaw")
        times.append([0.1, 1.9, 5.9, 9.9, 13.9, 17.9, 21.9, 25.9, 29.9, 33.9, 37.9, 41.9, 46.1])
        keys.append([0.0137641, 0.455531, 0.455531, 0.455531, 0.455531, 0.455531, 0.455531, 0.455531, 0.455531, 0.455531, 0.455531, 0.455531, 0.0137641])

        names.append("RElbowRoll")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([0.523087, 0.911062, 0.202458, 0.911062, 0.202458, 0.911062, 0.202458, 0.911062, 0.202458, 0.911062, 0.202458, 0.911062, 0.202458, 0.911062, 0.202458, 0.911062, 0.202458, 0.911062, 0.202458, 0.911062, 0.202458, 0.911062, 0.202458, 0.523087])

        names.append("RElbowYaw")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([1.23025, 0.434587, 1.83085, 0.434587, 1.83085, 0.434587, 1.83085, 0.434587, 1.83085, 0.434587, 1.83085, 0.434587, 1.83085, 0.434587, 1.83085, 0.434587, 1.83085, 0.434587, 1.83085, 0.434587, 1.83085, 0.434587, 1.83085, 1.23025])

        names.append("RHand")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([0.589631, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.89, 0.92, 0.589631])

        names.append("RShoulderPitch")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([1.55852, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, -1.08559, -1.12399, 1.55852])

        names.append("RShoulderRoll")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 46.1])
        keys.append([-0.14266, -0.261799, -0.76969, -0.261799, -0.76969, -0.261799, -0.76969, -0.261799, -0.76969, -0.261799, -0.76969, -0.261799, -0.76969, -0.261799, -0.76969, -0.261799, -0.76969, -0.261799, -0.76969, -0.261799, -0.76969, -0.261799, -0.76969, -0.14266])

        names.append("RWristYaw")
        times.append([0.1, 1.9, 5.9, 9.9, 13.9, 17.9, 21.9, 25.9, 29.9, 33.9, 37.9, 41.9, 46.1])
        keys.append([0.00149202, -0.455531, -0.455531, -0.455531, -0.455531, -0.455531, -0.455531, -0.455531, -0.455531, -0.455531, -0.455531, -0.455531, 0.00149202])

        try:
            self.motion_service.angleInterpolation(names, keys, times, True)
        except BaseException, err:
            print err

    def moveBody(self,motion_service):
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([0.01, 44.9])
        keys.append([-0.199418, -0.199418])

        names.append("HeadYaw")
        times.append([0.01, 44.9])
        keys.append([0, 0])

        names.append("HipPitch")
        times.append([0.01, 44.9])
        keys.append([-0.0322137, -0.0322137])

        names.append("HipRoll")
        times.append([0.01, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9, 37.9, 39.9, 41.9, 43.9, 44.9])
        keys.append([-0.0107379, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.0107379])

        names.append("KneePitch")
        times.append([0.01, 44.9])
        keys.append([0.0107379, 0.0107379])

        names.append("LElbowRoll")
        times.append([0.01, 44.9])
        keys.append([-0.523087, -0.523087])

        names.append("LElbowYaw")
        times.append([0.01, 44.9])
        keys.append([-1.23792, -1.23792])

        names.append("LHand")
        times.append([0.01, 44.9])
        keys.append([0.589631, 0.589631])

        names.append("LShoulderPitch")
        times.append([0.01, 44.9])
        keys.append([1.56159, 1.56159])

        names.append("LShoulderRoll")
        times.append([0.01, 44.9])
        keys.append([0.144194, 0.144194])

        names.append("LWristYaw")
        times.append([0.01, 44.9])
        keys.append([0.0137641, 0.0137641])

        names.append("RElbowRoll")
        times.append([0.01, 44.9])
        keys.append([0.523087, 0.523087])

        names.append("RElbowYaw")
        times.append([0.01, 44.9])
        keys.append([1.23025, 1.23025])

        names.append("RHand")
        times.append([0.01, 44.9])
        keys.append([0.589631, 0.589631])

        names.append("RShoulderPitch")
        times.append([0.01, 44.9])
        keys.append([1.55852, 1.55852])

        names.append("RShoulderRoll")
        times.append([0.01, 44.9])
        keys.append([-0.14266, -0.14266])

        names.append("RWristYaw")
        times.append([0.01, 44.9])
        keys.append([0.00149202, 0.00149202])

        try:
            self.motion_service.angleInterpolation(names, keys, times, True)
        except BaseException, err:
            print err
    def moveTest(self,motion_service):
        names = list()
        times = list()
        keys = list()

        names.append("HeadPitch")
        times.append([0.1, 35.9])
        keys.append([-0.199418, -0.199418])

        names.append("HeadYaw")
        times.append([0.1, 35.9])
        keys.append([0, 0])

        names.append("HipPitch")
        times.append([0.1, 35.9])
        keys.append([-0.0322137, -0.0322137])

        names.append("HipRoll")
        times.append([0.1, 1.9, 3.9, 5.9, 7.9, 9.9, 11.9, 13.9, 15.9, 17.9, 19.9, 21.9, 23.9, 25.9, 27.9, 29.9, 31.9, 33.9, 35.9])
        keys.append([-0.0107379, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, 0.349066, -0.349066, -0.0107379])

        names.append("KneePitch")
        times.append([0.1, 35.9])
        keys.append([0.0107379, 0.0107379])

        names.append("LElbowRoll")
        times.append([0.1, 35.9])
        keys.append([-0.523087, -0.523087])

        names.append("LElbowYaw")
        times.append([0.1, 35.9])
        keys.append([-1.23792, -1.23792])

        names.append("LHand")
        times.append([0.1, 35.9])
        keys.append([0.589631, 0.589631])

        names.append("LShoulderPitch")
        times.append([0.1, 35.9])
        keys.append([1.56159, 1.56159])

        names.append("LShoulderRoll")
        times.append([0.1, 35.9])
        keys.append([0.144194, 0.144194])

        names.append("LWristYaw")
        times.append([0.1, 35.9])
        keys.append([0.0137641, 0.0137641])

        names.append("RElbowRoll")
        times.append([0.1, 35.9])
        keys.append([0.523087, 0.523087])

        names.append("RElbowYaw")
        times.append([0.1, 35.9])
        keys.append([1.23025, 1.23025])

        names.append("RHand")
        times.append([0.1, 35.9])
        keys.append([0.589631, 0.589631])

        names.append("RShoulderPitch")
        times.append([0.1, 35.9])
        keys.append([1.55852, 1.55852])

        names.append("RShoulderRoll")
        times.append([0.1, 35.9])
        keys.append([-0.14266, -0.14266])

        names.append("RWristYaw")
        times.append([0.1, 35.9])
        keys.append([0.00149202, 0.00149202])
        try:
            self.motion_service.angleInterpolation(names, keys, times, True)
        except BaseException, err:
            print err
