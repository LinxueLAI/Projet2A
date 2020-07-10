from robot import *

pepper = Pepper("192.168.2.169", 9559)
# pepper.listen()
pepper.upload_file("aldialog.top")
# words =""
# while words =="":
#     words = pepper.listen_to(["Tu t'appelle comment?", "Oui", "Non"])
    # words = pepper.listen()
# print(str(words))
# pepper.autonomous_life_off()
# pepper.motion_service.wakeUp()

# detect the face the first time,then track the face
#pepper.trackFace()

# print(str(words))