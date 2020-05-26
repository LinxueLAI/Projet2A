from robot import Pepper
pepper = Pepper("192.168.2.169", 9559)
speed = 0.3
pepper.stand()
# pepper.motion_service
# Raise a hand to human
pepper.motion_service.angleInterpolationWithSpeed(["RShoulderPitch", "RWristYaw", "RHand"], [0.8, 2.5, 1.0], speed)

# Wait to touch a hand
while True:
    try:
        status = pepper.memory_service.getData("HandRightBackTouched")
        if status:
            pepper.motion_service.angleInterpolationWithSpeed("RHand", 0.0, speed*3)
            break
    except KeyboardInterrupt:
        break

# Get hand down
pepper.motion_service.angleInterpolationWithSpeed(["RShoulderPitch", "RWristYaw", "RHand"], [3.5, 0.00, 0.0], 1.0)#1.0=speed max?

# Throw a ball
# pepper.motion_service.angleInterpolationWithSpeed(["RShoulderPitch", "RWristYaw"], [1.0, 2.5], speed*2)
# pepper.motion_service.angleInterpolationWithSpeed("RHand", 1.0, speed*2)

# Reset position
pepper.stand()