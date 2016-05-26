import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import *
from morse.middleware.ros import ROSPublisher

class GpsROS(ROSPublisher):
    """ Publish the velocity of the acceleromter sensor.
    No angular information, only linear ones.
    """
    ros_class = PointStamped

    def default(self, ci='unused'):
        pointStamped = PointStamped()
        pointStamped.header = self.get_ros_header()

        # Fill twist-msg with the values from the sensor
        pointStamped.point.x = self.data['x']
        pointStamped.point.y = self.data['y']

        if self.data['x'] != float('nan'):
            self.publish(pointStamped)