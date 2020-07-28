from robot import *
pepper = Pepper("192.168.2.169", 9559)
# pepper.motion_service.wakeUp()
pepper.say("Pourriez-vous faire la meme mouvement comme moi?")
# print("Pourriez-vous faire la meme mouvement comme moi?")
# time.sleep(2)
# pepper.stand()
pepper.posture_service.goToPosture("StandInit",0.5)

pepper.wave(pepper.motion_service)
# pepper.say("Je peux faire taijiquan.")
# pepper.posture_service.goToPosture("StandInit", 1.0)
# pepper.taijiquan(pepper.motion_service)

pepper.say("Nous pouvons aussi faire des exercices comme sa!")
# print("Nous pouvons aussi faire des exercices comme sa!")
time.sleep(1)
# pepper.stand()
pepper.posture_service.goToPosture("StandInit",0.5)
pepper.wavewithhand(pepper.motion_service)

time.sleep(2)

pepper.say("Veuillez faire des mouvements comme moi.")
# print "Veuillez faire des mouvements comme moi"
# time.sleep(2)
# pepper.stand()
pepper.posture_service.goToPosture("StandInit",1.0)
# pepper.moveBody(pepper.motion_service)
pepper.moveTest(pepper.motion_service)

