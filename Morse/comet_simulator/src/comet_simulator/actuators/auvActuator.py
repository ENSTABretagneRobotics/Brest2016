import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator

from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.helpers.components import add_data, add_property

class Auvactuator(morse.core.actuator.Actuator):
    _name = "AuvActuator"

    add_data('vX', 0.0, 'double', 'Acceleration of the torpedo')
    add_data('gV', 0.0, 'double', 'Vertical orientation of the torpedo')
    add_data('gH', 0.0, 'double', 'Horizontal orientation of the torpedo')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        self.robot_parent.vX = self.local_data['vX']
        self.robot_parent.gV = self.local_data['gV']
        self.robot_parent.gH = self.local_data['gH']
        pass
