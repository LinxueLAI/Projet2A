from robot import *
from numpy import *
import argparse
import sys
import time
import almath
import math
def setDialog():
    pepper = Pepper("192.168.2.169", 9559)
    pepper.posture_service.goToPosture("Stand", 0.5)
    # pepper.navigation_service.stopExploration()
    # pepper.posture_service.getPostureList()
	# # self.pepper.dialog_service.setLanguage("French")
    pepper.motion_service.setAngles("HeadPitch", -0.1, 0.2)
    topic_content_1 = ('topic: ~example_topic_content()\n'
		           'language: frf\n'
		           'concept:(aliments) [fruits poulet oeufs boeuf]\n'
		           'proposal: veuillez me parler\n'
		           'u: (Bonjour) Bonjour humain.\n'
		           #'u: (Je [veux "voudrais"] {quelques} _~aliments) Daccord! Vous devez vraiment aimer les $1 .\n'
		           'u: (comment vas tu) Je vais bien merci, et toi? u1: (sa va) super u1: (sa ne va pas) Prend bien soin de toi.\n'
		           'u: (sa va?) Super!\n'
		           'u: (Pepper) Oui. Vous avez des questions?\n'
		           'u: (Jai deja porter le masque) Bien!\n'
		           'u: (quest-ce que tu mange?) Je suis un robot, je nai pas besoin de manger les aliments. Jai besoin de seulement electriciter.\n'
		           'u: (As-tu bien dormi?) Non! Tu as oublier de me couper et me charger!\n'
		           'u: (posez-moi une question) aimez-vous le poisson? u1: (oui) cest bon pour la santer u1: (non) je prefere la viande.\n'
		           'u: (parlez des animaux) avez-vous un chat ou un chien? u1: (jai un chat) super u1: (jai un chien) je prefere un chat u1: (non) ah. dommage.\n'
		           'u: (mon prenom est _*) Bonjour $1\n'
                   'u: ([e:FrontTactilTouched e:MiddleTactilTouched e:RearTactilTouched])Salut! ^break ^call(ALNavigationProxy.stopExploration()) ^call(ALRobotPosture.goToPosture("Stand", 0.5)) \n'
		           'u: (Quel est mon prenom?) ^first["votre prenom est $name" "je ne sais pas"] u1:(oui) rara u1: (non) OKay ^clear(name)\n'
		           'u: (Quelle est votre position?) ma position est ^break ^call(ALRobotPosture.getPosture()) c1:(_*) $1\n'
		           'u: ([e:faceDetected "Salut"]) Salut humain! ^break ^call(ALNavigationProxy.stopExploration()) ^break ^call(ALRobotPosture.goToPosture("Stand", 0.5)) vous voulez jouer un jeu avec moi?\n')
    #'u: ([e:FrontTactilTouched e:MiddleTactilTouched e:RearTactilTouched]) \n'
    print "the loaded topics: "+ str(pepper.dialog_service.getAllLoadedTopics())
	# # Loading the topics directly as text strings
    topic_name_1 = pepper.dialog_service.loadTopicContent(topic_content_1)

	# # Activating the loaded topics
    pepper.dialog_service.activateTopic(topic_name_1)

    pepper.dialog_service.subscribe('my_dialog_example')

    # trajectoire:

    pepper.motion_service.setAngles("HeadPitch", -0.1, 0.2)
    pepper.navigation_service.stopLocalization()
    path = "/home/nao/.local/share/Explorer/2015-06-19T204141.485Z.explo"
    pepper.navigation_service.loadExploration(str(path))
    pepper.navigation_service.startLocalization()
    home = pepper.navigation_service.getRobotPositionInMap()
    print "saved home position:"+str(home[0])
    print "go to point a : "+str(0)+","+str(0)
    pepper.navigation_service.navigateToInMap([0.,0.,0.0])
    print "reached a:"+str(pepper.navigation_service.getRobotPositionInMap()[0])
    
    p=array([[3.0,0.0],[3.0,2.0],[0.0,2.0],[0.0,0.0]])
    i = 0
    # time.sleep(5)
    while i < 1:
        print "go to point b : "+str(p[i][0])+","+str(p[i][1])
        pepper.navigation_service.navigateToInMap([int(p[i][0]),int(p[i][1]),0.0])
        print "fist navigation is finished"
        if (pepper.posture_service.getPostureFamily() == "Standing"):
            print "pepper arrive at: "+str(pepper.navigation_service.getRobotPositionInMap()[0])
            print "now we will do the detection of mask, here wait for 10 seconds"
            time.sleep(10)

        if(abs(pepper.navigation_service.getRobotPositionInMap()[0][0]-int(p[i][0]))>0.2) or (abs(pepper.navigation_service.getRobotPositionInMap()[0][1]-int(p[i][1]))>0.2):
            print "not reached ("+str(p[i][0])+","+str(p[i][1])+"),go to ("+str(p[i][0])+","+str(p[i][1])+")"
            pepper.navigation_service.navigateToInMap([int(p[i][0]),int(p[i][1]),0.0])
        else:
            print "arrived at "+str(pepper.navigation_service.getRobotPositionInMap()[0])
        # pepper.motion_service.waitUntilMoveIsFinished()
        print "reached b:"+str(pepper.navigation_service.getRobotPositionInMap()[0])
        i = i + 1

    # supprimer le dialogue
    pepper.dialog_service.unsubscribe('my_dialog_example')
    pepper.dialog_service.deactivateTopic(topic_name_1)
    pepper.dialog_service.unloadTopic(topic_name_1)

setDialog()