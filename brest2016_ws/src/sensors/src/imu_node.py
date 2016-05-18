#!/usr/bin/env python

import rospy
import serial
import string
import sys

from sensors.msg import YPR

rospy.init_node("razor_node")
# We only care about the most recent measurement, i.e. queue_size=1
pub = rospy.Publisher('imu', YPR, queue_size=1)
imuMsg = YPR()

default_port = '/dev/ttyUSB0'
# read  parameters
port = rospy.get_param('~port', default_port)

# Check your COM port and baud rate
rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=57600, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port " + port +
                 ". Did you specify the correct port in the launch file?")
    # exit
    sys.exit(0)

rospy.loginfo("Giving the razor IMU board 5 seconds to boot...")
rospy.sleep(5)  # Sleep for 5 seconds to wait for the board to boot

# start datastream
ser.write('#o1' + chr(13))

rospy.loginfo("Flushing first 200 IMU entries...")
for x in range(0, 200):
    line = ser.readline()
rospy.loginfo("Publishing IMU data...")

while not rospy.is_shutdown():
    line = ser.readline()
    line = line.replace("#YPR=", "")   # Delete "#YPR="
    words = string.split(line, ",")    # Fields split

    imuMsg.Y = float(words[0])
    imuMsg.P = float(words[1])
    imuMsg.R = float(words[2])

    pub.publish(imuMsg)

ser.close
