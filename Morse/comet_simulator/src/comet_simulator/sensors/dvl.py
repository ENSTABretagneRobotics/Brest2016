import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

import random
from math import *

class Dvl(morse.core.sensor.Sensor):
    _name = "Dvl"
    _short_desc = "Measure the velocity of an object"

    _v_resolution = 1e-4 # 0.1 mm/s
    _sigma = 2e-3 # +-2mm/s
    _sigma1 = 0.005 # +-0.50%

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('vX', float, 'Vector', 'Velocity')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)
        logger.info('Component initialized')

    def default_action(self):
        v = self.robot_parent._v
        v_bruit = random.gauss(v, v*self._sigma1 + self._sigma) # +-0.50% , +-2mm/s

        v_resolution = round(v_bruit / self._v_resolution) * self._v_resolution

        self.local_data['vX'] = v_resolution
