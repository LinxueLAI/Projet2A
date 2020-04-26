from naoqi import ALProxy
import argparse
import almath

positionErrorThresholdPos = 0.01
positionErrorThresholdAng = 0.03
ip = "192.168.2.169"
motion = ALProxy("ALMotion", ip, 9559)         
tts = ALProxy("ALTextToSpeech", ip, 9559)      
posture = ALProxy("ALRobotPosture", ip, 9559)  
#motion.wakeUp()
posture.goToPosture("StandInit", 0.5)
motion.setStiffnesses("Body", 1.0)
motion.moveInit()
i = 1
while i>0 :
    initPosition = almath.Pose2D(motion.getRobotPosition(True))
    print(motion.getRobotPosition(True))
    targetDistance = almath.Pose2D(("Distance X (m)"),getParameter("Distance Y (m)"),getParameter("Theta (rad)"))
    print("X = "+ getParameter("Distance X (m)"))
    print("Y = "+ getParameter("Distance Y (m)"))
    print("theta = "+ getParameter("Theta (rad)"))
    expectedEndPosition = initPosition * targetDistance
    print("expectedEndPosition = "+expectedEndPosition)
    enableArms = getParameter("Arms movement enabled")
    motion.setMoveArmsEnabled(enableArms, enableArms)
    motion.moveTo(getParameter("Distance X (m)"),getParameter("Distance Y (m)"),getParameter("Theta (rad)"))
    # The move is finished so output
    realEndPosition = almath.Pose2D(motion.getRobotPosition(False))
    positionError = realEndPosition.diff(expectedEndPosition)
    positionError.theta = almath.modulo2PI(positionError.theta)
    if (abs(positionError.x) < positionErrorThresholdPos and abs(positionError.y) < positionErrorThresholdPos and abs(positionError.theta) < self.positionErrorThresholdAng):
        onArrivedAtDestination()
    else:
        onStoppedBeforeArriving(positionError.toVector())
#id3 = motion.post.moveTo(1.9, 0.0, 0.5)
#id3 = motion.post.moveTo(2.0, 0.0, 0.5)
#tts.say("Je continue.")
#motion.wait(id3, 0)
#tts.say("OK,Je stop.")
#motion.rest()
