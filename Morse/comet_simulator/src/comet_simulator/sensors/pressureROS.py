import rospy
from geometry_msgs.msg import *
from morse.middleware.ros import ROSPublisher

class PressureROS(ROSPublisher):

    ros_class = PointStamped

    def default(self, ci='unused'):
        pointStamped = PointStamped()
        pointStamped.header = self.get_ros_header()

        # Fill twist-msg with the values from the sensor
        pointStamped.point.z = self.data['z']
        self.publish(pointStamped)

