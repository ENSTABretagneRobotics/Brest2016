import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

import random
from math import *

class Imu(morse.core.sensor.Sensor):
    
    _name = "Imu"
    _sigma_roll = 0.2 * pi/180.0
    _sigma_pitch = 0.2 * pi/180.0
    _sigma_heading = 0.5 * pi/180.0

    add_data('yaw', 0.0, 'double', 'yaw of the robot')
    add_data('pitch', 0.0, 'double', 'pithc y of the robot')
    add_data('roll', 0.0, 'double', 'roll z of the robot')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):

        self.local_data['yaw'] =    random.gauss(self.robot_parent.yaw, self._sigma_heading)
        self.local_data['pitch'] =  random.gauss(self.robot_parent.pitch, self._sigma_pitch)
        self.local_data['roll'] =   random.gauss(self.robot_parent.roll, self._sigma_roll)
