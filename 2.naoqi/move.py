from naoqi import ALProxy
motion1 = ALProxy("ALMotion", "192.168.2.169", 9559)
motion1.setStiffnesses("Body", 1.0)

motion1.moveInit()
motion1.moveTo(-0.2, 0, 0)