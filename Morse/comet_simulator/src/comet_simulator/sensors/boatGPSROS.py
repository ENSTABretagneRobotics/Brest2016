import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import *
from morse.middleware.ros import ROSPublisher

class BoatGPSROS_pose(ROSPublisher):
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
        self.publish(pointStamped)

class BoatGPSROS_velocity(ROSPublisher):
    """ Publish the velocity of the acceleromter sensor.
    No angular information, only linear ones.
    """
    ros_class = TwistStamped

    def default(self, ci='unused'):
        twistStamped = TwistStamped()
        twistStamped.header = self.get_ros_header()

        # Fill twist-msg with the values from the sensor
        twistStamped.twist.linear.x = self.data['vX']

        self.publish(twistStamped)