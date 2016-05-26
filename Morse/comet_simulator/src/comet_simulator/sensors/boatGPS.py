import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property

from math import *

class Boatgps(morse.core.sensor.Sensor):
    _name = "BoatGps"
    _short_desc = "gps"

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('x', 0.0, 'float', 'x coordinate in world frame in meters')
    add_data('y', 0.0, 'float', 'y coordinate in world frame in meters')
    add_data('vX', 0.0, 'float', 'vx speed in the sensor frame in meters')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):

        self.local_data['x'] = self.robot_parent.x # distance along X in world coordinates
        self.local_data['y'] = self.robot_parent.y # distance along Y in world coordinates
        self.local_data['vX'] = self.robot_parent._v
