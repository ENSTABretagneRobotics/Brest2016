import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor

from morse.core import blenderapi
from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property
import random
from math import *

class Range(morse.core.sensor.Sensor):
    _name = "Range"
    _short_desc = "Measure the distances to all the robots in the scene"
    _sigma = 0.3 # m
    _pourcentage = 0.01 # 1% de la mesure
    _resolution = 1e-3 # 1 mm

    add_data('ranges', {}, 'dict', 'All the distances to all the robots in the scene')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)
        logger.info('Component initialized')

    def default_action(self):
        self.local_data['ranges']={} # empty the dict
        parent = self.robot_parent.bge_object
        # Loop through all the objects in the scene
        for obj in blenderapi.scene().objects:
          # Skip distance to self
          if parent != obj:
            distance = self._measure_distance_to_object (parent, obj)

            distance_bruit = random.gauss(distance, self._sigma + self._pourcentage*distance)

            self.local_data['ranges'][obj.name] = round(distance_bruit / self._resolution) * self._resolution
            #logger.info('Distance measured from ' + str(obj.name) + ' = ' + str(distance))

    def _measure_distance_to_object(self, own_robot, target_object):
        distance, globalVector, localVector = own_robot.getVectTo(target_object)
        return distance