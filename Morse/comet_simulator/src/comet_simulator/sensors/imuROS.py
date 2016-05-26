import roslib; roslib.load_manifest('geometry_msgs')

from geometry_msgs.msg import *
from morse.middleware.ros import ROSPublisher

class ImuROS(ROSPublisher):
    ros_class = Vector3

    def default(self, ci='unused'):
        vector3 = Vector3()
        #vector3.header = self.get_ros_header()

        vector3.z = self.data['yaw']


        self.publish(vector3)