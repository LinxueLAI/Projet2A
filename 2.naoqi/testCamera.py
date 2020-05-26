from robot import *
# import config
import argparse
import sys
import time
pepper = Pepper("192.168.2.169", 9559)

# Test camera ## marche bien (dynamique)
pepper.subscribe_camera("camera_top", 2, 30)
while True:
    image = pepper.get_camera_frame(show=False)#true?
    cv2.imshow("frame", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pepper.unsubscribe_camera()
cv2.destroyAllWindows()

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
