#!/usr/bin/env	python
import rospy
import serial
import ais
import re
from sensors.msg import GPRMC


rospy.init_node('GPRMC_publisher')

pub = rospy.Publisher('GPRMC', GPRMC)
rate = rospy.rate(2)

ser = serial.Serial("/dev/ttyUSB0", 4800)

while not rospy.is_shutdown():

    data = ser.readline()
    data = sata.split(',')
    data12 = data[-1].split('*')

    if data[0] = '$GPRMC':
        msg = GPRMC()
        msg.type = data[0]
        msg.heure = data[1]
        msg.etat = data[2]
        msg.latitude = data[3]
        msg.indiLatitude = data[4]
        msg.longitude = data[5]
        msg.indiLongitude = data[6]
        msg.vitesse = data[7]
        msg.route = data[8]
        msg.date = data[9]
        msg.declinMagnetique = data[10]
        msg.sensDeclinMagnetique = data[11]
        msg.checksum = data12[1]
        pub.publish(msg)
        rate.sleep()