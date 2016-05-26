import roslib; roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import *
from morse.middleware.ros import ROSPublisher

class RangeROS(ROSPublisher):

    ros_class = LaserScan

    def default(self, ci='unused'):
        range_msg = LaserScan()
        range_msg.header = self.get_ros_header()
        range_msg.ranges = [0.0,0.0,0.0]
        
        for key, value in self.data['ranges'].items():
            if key == "auv1":
                range_msg.ranges[0] = value
            if key == "auv2":
                range_msg.ranges[1] = value
            if key == "boat":
                range_msg.ranges[2] = value
        self.publish(range_msg)
