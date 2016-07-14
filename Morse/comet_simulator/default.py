#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <comet_simulator> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from comet_simulator.builder.robots import *
from comet_simulator.builder.actuators import *
from comet_simulator.builder.sensors import *

#    BOAT     ###
boat = Boat('boat')
boat.translate(-30.0, -60.0, 0.0)

# SENSORS #
kinect = Kinect()
kinect.translate(0, 0, 3)
kinect.rotate(0, 0, 0)
kinect.add_stream('ros')
boat.append(kinect)

# Add a keyboard controller to move the robot with arrow keys.
keyboard = Keyboard()
boat.append(keyboard)
keyboard.properties(ControlType='Position')

# ENVIRONMENT

# set 'fastmode' to True to switch to wireframe mode
env = Environment(
    # '/home/brest2016/Desktop/Brest_2016/brest-git/Brest2016/Morse/comet_simulator/data/comet_simulator/environments/costal_env.blend', fastmode=False)
    '/home/ejalaa12/Desktop/Brest2016/Morse/environments/water-1o-3w.blend', fastmode=False)
env.set_camera_location([0.0, 0.0, 200.0])
env.set_camera_rotation([0, 0.0, 0.0])
env.set_camera_clip(clip_end=2000)
