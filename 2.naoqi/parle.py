from naoqi import ALProxy
tts = ALProxy("ALTextToSpeech", "169.254.247.23", 9559)   
tts.say("Bonjour!")