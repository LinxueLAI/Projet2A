from robotDetector import *

pepper = Pepper("192.168.2.169", 9559)

pepper.subscribe_camera("camera_top", 2, 30)

# load our serialized face detector model from disk
print("[INFO] loading face detector model...")
prototxtPath = os.path.sep.join(["face_detector", "deploy.prototxt"])
weightsPath = os.path.sep.join(["face_detector",
	"res10_300x300_ssd_iter_140000.caffemodel"])
faceNet = cv2.dnn.readNet(prototxtPath, weightsPath)
label = ""
# load the face mask detector model from disk
print("[INFO] loading face mask detector model...")
maskNet = load_model("mask_detector.model")

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")

# loop over the frames from the video stream
while True:
	frame = pepper.get_camera_frame(show=False)

	# detect faces in the frame and determine if they are wearing a
	# face mask or not
	(locs, preds) = pepper.detect_and_predict_mask(frame, faceNet, maskNet)

	# loop over the detected face locations and their corresponding
	# locations
	for (box, pred) in zip(locs, preds):
		# unpack the bounding box and predictions
		(startX, startY, endX, endY) = box
		(mask, withoutMask) = pred

		# determine the class label and color we'll use to draw
		# the bounding box and text
		label = "Mask" if mask > withoutMask else "No Mask"
		color = (0, 255, 0) if label == "Mask" else (0, 0, 255)
		if label=="No Mask":
			#pepper.tracker_service.registerTarget("Face", 0.15)
			#pepper.tracker_service.setMode("Move")
			#pepper.tracker_service.track("Face")
			pepper.say("Portez le masque si vous plait.")
			time.sleep(4)
			break
		elif label=="Mask":
			pepper.say("C'est bien, vous avez porter le masque.")
			time.sleep(4)
			break
		# include the probability in the label
		label = "{}: {:.2f}%".format(label, max(mask, withoutMask) * 100)

		# display the label and bounding box rectangle on the output
		# frame
		cv2.putText(frame, label, (startX, startY - 10),
			cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
		cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)

	# show the output frame
	cv2.imshow("Frame", frame)

	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

# do a bit of cleanup
cv2.destroyAllWindows()

