from naoqi import ALProxy
motion = ALProxy("ALMotion", "192.168.2.169", 9559)
tts = ALProxy("ALTextToSpeech", "192.168.2.169", 9559)
motion.moveInit()
tts.say("Bonjour!")
motion.post.moveTo(0.2, 0, 0.5)