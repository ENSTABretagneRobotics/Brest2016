import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

import random
from math import *

class Pressure(morse.core.sensor.Sensor):
    _name = "Pressure"
    _short_desc = "Measure the pressure"
    _sigma = 0.1  # 10 cm
    _resolution = 1e-3 # 1 mm
    
    #_precision = 0.2
    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('z', 0.0, 'value', 'depth of the robot')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)
        logger.info('Component initialized')

    def default_action(self):
        z_bruit = - random.gauss(self.robot_parent.z, self.robot_parent.z + self._sigma)
        self.local_data['z'] =  round(z_bruit / self._resolution) * self._resolution

