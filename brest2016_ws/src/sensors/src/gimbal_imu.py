#!/usr/bin/env python
import rospy

from sensors.msg import YPR

rospy.init_node("gimbal_imu")

pub = rospy.Publisher('laser_orientation', YPR, queue_size=1)
imuMsg = YPR()

# Gimbal Port
default_port = '/dev/ttyUSB0'
# read  parameters
port = rospy.get_param('~port', default_port)

while not rospy.is_shutdown():
    pub.publish(imuMsg)
