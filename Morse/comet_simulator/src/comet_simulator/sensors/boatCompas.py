import logging
logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

import random
from math import *


class Boatcompas(morse.core.sensor.Sensor):
    add_data('yaw', 0.0, 'double', 'yaw of the boat')
    add_data('yaw_dot', 0.0, 'double', 'yaw dot of the boat')

    _sigma_yaw = 2.0 * pi / 180.0       # rad
    _sigma_yaw_dot = 1.0 * pi / 180.0   # rad/s
    _yaw_resolution = 0.1 * pi / 180.0
    _yaw_dot_resolution = 0.1 * pi / 180.0

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)
        self.previous_yaw = 0.0
        logger.info('Component initialized')

    def default_action(self):

        yaw_bruit = random.gauss(self.robot_parent.yaw, self._sigma_yaw)
        yaw_dot_bruit = random.gauss(
            self.robot_parent.dyaw, self._sigma_yaw_dot)

        self.local_data['yaw'] = round(
            yaw_bruit / self._yaw_resolution) * self._yaw_resolution
        self.local_data['yaw_dot'] = round(
            yaw_dot_bruit / self._yaw_dot_resolution) * self._yaw_dot_resolution
