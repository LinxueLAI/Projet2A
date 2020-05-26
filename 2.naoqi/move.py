from naoqi import ALProxy
motion1 = ALProxy("ALMotion", "192.168.2.169", 9559)
motion1.setStiffnesses("Body", 1.0)
motion1.wakeUp()
motion1.moveInit()
motion1.moveTo(4.0, 0, 0)
motion1.moveTo(0.0, 0, 3.1415926)
motion1.moveTo(4.0, 0, 0)
motion1.moveTo(0.0, 0, 3.1415926)
