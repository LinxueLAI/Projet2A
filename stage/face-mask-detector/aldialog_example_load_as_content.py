#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Using ALDialog Methods"""

import qi
import argparse
import sys

def main(session):
    # """
    # This example uses ALDialog methods.
    # It's a short dialog session with two topics.
    # """
    # Getting the service ALDialog
    ALDialog = session.service("ALDialog")
    ALDialog.setLanguage("French")

    # writing topics' qichat code as text strings (end-of-line characters are important!)
    topic_content_1 = ('topic: ~example_topic_content()\n'
                       'language: frf\n'
                       'concept:(aliments) [fruits poulet oeufs boeuf]\n'
                       'proposal: veuillez me parler\n'
                       'u: (Bonjour) Bonjour humain.\n'
                       'u: (Je [veux "voudrais"] {quelques} _~aliments) Daccord! Vous devez vraiment aimer les $1 .\n'
                       'u: (comment vas tu aujourd hui) Je vais bien merci, et toi?\n'
                       'u: (ça va?) Super!\n'
                       'u: (Pepper) Oui. Voua avez des questions?\n'
                       'u: (Jai déjà porté le masque) Bien!\n'
                       'u: (quest-ce que tu mange?) Je suis un robot, je nai pas besoin de manger les aliments. Jai besoin de seulement éléctricité.\n'
                       'u: (As-tu bien dormi?) Non! Tu as oublie de me couper et me charger!\n'
                       'u: (posez-moi une question) aimez-vous le poisson? u1: (oui) ce est bon pour la santé u1: (non) je préfère la viande.\n'
                       'u: (parlez des animaux) avez-vous un chat ou un chien? u1: (jai un chat) super u1: (jai un chien) je préfère un chat u1: (non) ah. dommage.\n'
                       'u: (mon prénom est _*) Bonjour $name=$1\n'
                       'u: (Quel est mon prénom?) ^first["votre prénom est $name" "je ne sais pas"] u1:(oui) rara u1: (non) OK ^clear(name)\n'
                       'u: (Quelle est votre position?) ma position est ^break ^call(ALRobotPosture.getPosture()) c1:(_*) $1'
                       'u: ([e:FrontTactilTouched e:MiddleTactilTouched e:RearTactilTouched]) Tu as touché ma tête!\n'
                       'u: ([e:faceDetected "Salut"]) Salut humain!\n')

    topic_content_2 = ('topic: ~dummy_topic()\n'
                       'language: frf\n'
                       'u:(test) [a b "c d" "e f g"]\n')

    # # Loading the topics directly as text strings
    topic_name_1 = ALDialog.loadTopicContent(topic_content_1)
    topic_name_2 = ALDialog.loadTopicContent(topic_content_2)

    # # Activating the loaded topics
    ALDialog.activateTopic(topic_name_1)
    ALDialog.activateTopic(topic_name_2)

    # # Starting the dialog engine - we need to type an arbitrary string as the identifier
    # # We subscribe only ONCE, regardless of the number of topics we have activated
    ALDialog.subscribe('my_dialog_example')

    try:
        raw_input("\nSpeak to the robot using rules from both the activated topics. Press Enter when finished:")
    finally:
        # stopping the dialog engine
        ALDialog.unsubscribe('my_dialog_example')

        # Deactivating all topics
        ALDialog.deactivateTopic(topic_name_1)
        ALDialog.deactivateTopic(topic_name_2)

        # now that the dialog engine is stopped and there are no more activated topics,
        # we can unload all topics and free the associated memory
        ALDialog.unloadTopic(topic_name_1)
        ALDialog.unloadTopic(topic_name_2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.2.169",
                        help="Robot's IP address. If on a robot or a local Naoqi - use '127.0.0.1' (this is the default value).")
    parser.add_argument("--port", type=int, default=9559,
                        help="port number, the default value is OK in most cases")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://{}:{}".format(args.ip, args.port))
    except RuntimeError:
        print ("\nCan't connect to Naoqi at IP {} (port {}).\nPlease check your script's arguments."
               " Run with -h option for help.\n".format(args.ip, args.port))
        sys.exit(1)
    main(session)