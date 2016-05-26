import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor
import time

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

import random
from math import *

class Gps(morse.core.sensor.Sensor):
    _name = "Gps"
    _short_desc = "gps"
    _gps_depth = 0.2 # 20 cm depth
    _gps_time_hang = 5.0

    _last_time = 0.0

    _sigma = 3.0 # 2 m
    _resolution = 0.01 # 1 cm

    add_data('x', 0.0, 'float', 'x coordinate in world frame in meters')
    add_data('y', 0.0, 'float', 'y coordinate in world frame in meters')

    add_data('time_surface', 0.0, 'float', 'Time (in s) at depth < 20 cm')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        self.local_data['x'] = float('nan')
        self.local_data['y'] = float('nan')
        self.local_data['vX'] = float('nan')

        #current_time = self.gettime()
        current_time = time.time()
        dt = (current_time - self._last_time)
        self._last_time = current_time

        if self.robot_parent.z < self._gps_depth:
            self.local_data['time_surface'] += dt
            if self.local_data['time_surface'] > self._gps_time_hang:
                x_bruit = random.gauss(self.robot_parent.x, self._sigma)
                y_bruit = random.gauss(self.robot_parent.y, self._sigma)

                self.local_data['x'] = round(x_bruit / self._resolution)*self._resolution
                self.local_data['y'] = round(y_bruit / self._resolution)*self._resolution
        else:
            self.local_data['time_surface'] = 0.0