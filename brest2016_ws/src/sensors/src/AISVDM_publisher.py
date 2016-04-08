#!/usr/bin/env  python
import serial
import rospy
import ais.stream
import ais
from sensors.msg import AISVDM


rospy.init_node('AISVDM_publisher')

pub = rospy.Publisher('AISVDM', AISVDM)
rate = rospy.Rate(2)
ser = serial.Serial("/dev/ttyACM0")

# with open("/dev/ttyACM0") as f:
#     for msg in ais.stream.decode(f):
#         print msg

while not rospy.is_shutdown():

    data = ser.readline()
    data = data.split(',')
    if (data[0] == '!AIVDM' and data[1] == 1):

        data = ais.decode(data[5])

        msg = AISVDM()
        msg.heure = data['utc_hour']
        msg.minute = data['utc_minut']
        msg.numMMSI = data['mmsi']
        msg.latitude = data['x']
        msg.longitude = data['y']
        msg.vitesse = data['sog']
        msg.cap = data['cog']
        msg.vitesseRot = data['rot']
        msg.statut = data['special_manoeuvre']

        print data
        pub.publish(msg)
    rate.sleep()
