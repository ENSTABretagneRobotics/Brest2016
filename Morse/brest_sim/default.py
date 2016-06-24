#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <brest_sim> environment

Feel free to edit this template as you like!
"""

from morse.builder import *

# Add the MORSE mascott, MORSY.
# Out-the-box available robots are listed here:
# http://www.openrobots.org/morse/doc/stable/components_library.html
#
# 'morse add robot <name> brest_sim' can help you to build custom robots.
# robot = Morsy()
robot = ATRV()

obstacle = ATRV()

# The list of the main methods to manipulate your components
# is here:
# http://www.openrobots.org/morse/doc/stable/user/builder_overview.html
robot.translate(1.0, 0.0, 0.0)
robot.rotate(0.0, 0.0, 3.5)

obstacle.translate(-2.0, 0.0, 0.0)
obstacle.rotate(0.0, 0.0, 0)

# Add a motion controller
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
#
# 'morse add actuator <name> brest_sim' can help you with the creation of
# a custom actuator.
motion = MotionVW()
robot.append(motion)
motion.add_stream('ros')
# obstacle.append(motion)


# Add a keyboard controller to move the robot with arrow keys.
keyboard = Keyboard()
robot.append(keyboard)
# obstacle.append(keyboard)
keyboard.properties(Speed=1.0, ControlType='Position')
# keyboard.properties(ControlType='Velocity')

# Add a pose sensor that exports the current location and orientation
# of the robot in the world frame
# Check here the other available actuators:
# http://www.openrobots.org/morse/doc/stable/components_library.html#sensors
#
# 'morse add sensor <name> brest_sim' can help you with the creation
# of a custom sensor.
pose1 = Pose()
pose1.name = 'pose'
pose2 = Pose()
pose2.name = 'pose'
robot.append(pose1)
obstacle.append(pose2)
pose1.add_stream('ros')
pose2.add_stream('ros')


kinect = Kinect()
kinect.translate(0, 0, 3)
kinect.rotate(0, 0, 0)
kinect.add_stream('ros')
robot.append(kinect)

sick = SickLDMRS()
sick.translate(0, 0, 2)
sick.properties(Visible_arc=True)
sick.properties(resolution=1.0)
sick.properties(scan_window=1)
sick.properties(laser_range=30.0)
sick.properties(layers=1)
sick.properties(layer_separation=0.8)
sick.properties(layer_offset=0.25)
sick.frequency(1.0)

sick.add_stream('ros')
robot.append(sick)

# To ease development and debugging, we add a socket interface to our robot.
#
# Check here: http://www.openrobots.org/morse/doc/stable/user/integration.html
# the other available interfaces (like ROS, YARP...)
robot.add_default_interface('socket')


# set 'fastmode' to True to switch to wireframe mode
env_path = 'outdoors'
# env_path = '/home/brest2016/Desktop/Brest_2016/brest-git/Brest2016/Morse/environments/water-1o-3w.blend'
# env_path = 'indoors-1/indoor-1'
env = Environment(env_path, fastmode=False)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])
