from robot import *

pepper = Pepper("192.168.2.169", 9559)
# pepper.autonomous_life_on()
# pepper.upload_file("./masque.jpg")
# detect the face the first time,then track the face
#pepper.trackFace()

pepper.dialog_service.setLanguage("English")
