from robot import *
# import config
import argparse
import sys
import time
pepper = Pepper("192.168.2.169", 9559)

# # Get robot into default standing position known as `StandInit` or `Stand`, here is 'stand', param = 0.5
# pepper.stand()

# # Get robot into default resting position know as `Crouch` param = 0.5
# pepper.rest()

# # Point end-effector in cartesian space
# pepper.point_at(1.0, 1.0, 0.0, "RArm", 0)

# # Move forward with certain speed
# pepper.move_forward(0.5)

# # Stop robot
pepper.stop_moving()

# # Text to speech
# pepper.say("Bonjour!")

# # Restart robot (it takes several minutes)
# pepper.restart_robot()

# # Turn off the robot completely
# pepper.shutdown_robot()

# # Switch autonomous life off
# pepper.autonomous_life_off()

# # Switch autonomous life on
# pepper.autonomous_life_on()

# # Track a object with a given object type and diameter. diameter = 0.15 
# pepper.track_object("Face","Arms")

# # Start exploration mode when robot it performing a SLAM in specified radius. Then it saves a map into robot into its default folder.
# pepper.exploration_mode(2)

# # Shows a map from robot based on previously loaded one or explicit exploration of the scene. It can be viewed in the computer by OpenCV.
# pepper.show_map(False,None)#on_robot=False, remote_ip=None

# # Localize a robot in a map
# pepper.robot_localization()

# # Stop localization of the robot
# pepper.stop_localization

# # Subscribe to a camera service. We need to subscribe a camera before we reach a images from it.
# pepper.subscribe_camera("camera_top", 2, 30)

# # Unsubscribe to camera after we don't need it
# pepper.unsubscribe_camera()

# # Get camera frame from subscribed camera link.
# pepper.get_camera_frame(show=False)

# # Get depth frame from subscribed camera link.
# pepper.get_depth_frame(show=False)

# # Show image from camera with SpeechToText annotation on the robot tablet
# pepper.show_tablet_camera("camera top")

# # Set security distance. Lower distance for passing doors etc.
# pepper.set_security_distance(distance=0.05)

# # Look down/Look up/Put head into default position in 'StandInit' pose
# pepper.move_head_up()
# pepper.move_head_down()
# pepper.move_head_default()

# # Move a robot into circle for specified time
# pepper.move_to_circle(clockwise=True, t=5)#clockwise

# # Blink eyes with defined color
# pepper.blink_eyes([255, 0, 0])

# # Turn off the LEDs in robot's eyes
# pepper.turn_off_leds()# pepper.blink_eyes([0,0,0])

# # Navigate robot in map based on exploration mode or load previously mapped enviroment.
# pepper.robot_localization()
# pepper.navigate_to(1.0,0.3)

# # Unsubscribe a end-effector after tracking some object
# pepper.unsubscribe_effector()

# # Complex movement for choosing a random people.
# pepper.pick_a_volunteer()

# # Gets all face properties from the tracked face in front of the robot.
# pepper.get_face_properties()

# # Listen and match the vocabulary which is passed as parameter.
# pepper.listen_to(["what color is the sky", "yes", "no"])

# # Wildcard speech recognition via internal Pepper engine
# # pepper.listen()

# # Ask for question and then robot will say first two sentences from Wikipedia
# pepepr.ask_wikipedia()

# # Change current name of the robot
# # pepper.rename_robot()

# # Gets a current name of the robot
# pepper.get_robot_name()

# # Close or open hand
# pepper.hand("right",close=True)

# # dance
# pepper.dance(pepper.motion_service)

