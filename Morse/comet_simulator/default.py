#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <comet_simulator> environment

Feel free to edit this template as you like!
"""

from morse.builder import *
from comet_simulator.builder.robots import *
from comet_simulator.builder.actuators import *
from comet_simulator.builder.sensors import *

###################    BOAT     ###################
boat = Boat('boat')
boat.translate(-60.0,  -60.0, 0.0)


################### ENVIRONMENT ###################

# set 'fastmode' to True to switch to wireframe mode
env = Environment(
    '/home/benoit/Desktop/Brest2016/Morse/environments/water-1o-3w.blend', fastmode=False)
env.set_camera_location([0.0, 0.0, 300.0])
env.set_camera_rotation([0, -0., -0])
env.set_camera_clip(clip_end=2000)
