import logging; logger = logging.getLogger("morse." + __name__)
import roslib ; roslib.load_manifest('geometry_msgs')

from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSSubscriber

class BoatActuatorROS(ROSSubscriber):

    ros_class = Twist

    def update(self, message):
        self.data['vX'] = message.linear.x
        self.data['wZ'] = message.angular.z