from robot import *
# import config
from robot_detect_image import *
import argparse
import sys
import time
pepper = Pepper("192.168.2.169", 9559)
#pepper.track_object("Face","LArm",0.15)
#pepper.autonomous_life_off()
# Test camera ## marche bien (dynamique)
pepper.subscribe_camera("camera_top", 2, 30)
while True:
    image = pepper.get_camera_frame(show=False)#true?
    cv2.imshow("frame", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#pepper.unsubscribe_camera()
#cv2.destroyAllWindows()
#pepper.say("Je suis en train d'analyser...")
#label = detect_image()
#text="C'est bien, tu as porte une masque." if label=="Mask" else "Porte le masque si te plait."
#pepper.say(text)

## Test speech function
# image = pepper.get_camera_frame(True)
# pepper.unsubscribe_camera()
# while True:
#     pepper.say("Give me a question")
#     try:
#         pepper.ask_wikipedia()
#     except Exception as error:
#         print(error)
#         pepper.say("I am not sure what to say")
