import roslib; roslib.load_manifest('std_msgs')
from std_msgs.msg import *
from morse.middleware.ros import ROSPublisher

class BoatCompasROS(ROSPublisher):
    ros_class = Float32MultiArray

    def default(self, ci='unused'):
        compas = Float32MultiArray()
        compas.data = [0.0, 0.0]
        #pointStamped.header = self.get_ros_header()

        # Fill twist-msg with the values from the sensor
        compas.data[0] = self.data['yaw']
        compas.data[1] = self.data['yaw_dot']

        self.publish(compas)