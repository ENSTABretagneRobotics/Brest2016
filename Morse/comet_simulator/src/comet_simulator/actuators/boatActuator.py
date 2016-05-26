import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator

from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.helpers.components import add_data, add_property

class Boatactuator(morse.core.actuator.Actuator):

    _name = "Boatactuator"
    _short_desc = "Exposes the inputs u1 (acceleration), u2 (rudder) of the boat"

    add_data('vX',0,'double','Acceleration of the boat')
    add_data('wZ',0,'double','Rudder of the boat')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        morse.core.actuator.Actuator.__init__(self, obj, parent)

    def default_action(self):
        # You should copy the local_data values to the parent_robot
        #logger.info('BoatActuator is not implemented')
        self.robot_parent.vX = self.local_data['vX']
        self.robot_parent.wZ = self.local_data['wZ']
        pass
