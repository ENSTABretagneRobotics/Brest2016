#!/usr/bin/env  python
import rospy
import serial
from sensors.msg import AISVDM


rospy.init_node('AISVDM_publisher')

pub = rospy.Publisher('AISVDM', AISVDM)
rate = rospy.rate(2)

ser = serial.Serial("/dev/ttyUSB1")

while not rospy.is_shutdown():

    data = ser.readline()
    data = data.split(',')

    msg = AISVDM()
    msg.heure = data[0]
    msg.numMMSI = data[1]
    msg.latitude = data[2]
    msg.longitude = data[3]
    msg.vitesse = data[4]
    msg.cap = data[5]
    msg.vitesseFond = data[6]
    msg.vitesseRot = data[7]
    msg.statut = data[8]

    pub.publish(msg)
    rate.sleep()
