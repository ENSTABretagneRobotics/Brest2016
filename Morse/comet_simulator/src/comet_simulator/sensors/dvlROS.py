import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import *
from morse.middleware.ros import ROSPublisher

class DvlROS(ROSPublisher):
    ros_class = TwistStamped

    def default(self, ci='unused'):
        twistStamped = TwistStamped()
        twistStamped.header = self.get_ros_header()

        # Fill twist-msg with the values from the sensor
        twistStamped.twist.linear.x = self.data['vX']

        self.publish(twistStamped)