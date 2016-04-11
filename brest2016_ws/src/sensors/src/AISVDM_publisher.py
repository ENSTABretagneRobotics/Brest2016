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


def publish(data):
    msg = AISVDM()
    msg.heure = int(data['utc_hour'])
    msg.minute = int(data['utc_min'])
    msg.numMMSI = int(data['mmsi'])
    msg.latitude = float(data['x'])
    msg.longitude = float(data['y'])
    msg.vitesse = float(data['sog'])
    msg.cap = float(data['cog'])
    msg.vitesseRot = float(data['rot'])
    msg.statut = float(data['special_manoeuvre'])

    print data
    pub.publish(msg)
    rate.sleep()

buff = ''  # for first part of 2-lines msgs
while not rospy.is_shutdown():

    data = ser.readline()
    data = data.split(',')
    # msg in one line
    if data[0] == '!AIVDM' and data[1] == '1':
        publish(ais.decode(data[5], 0))

    # msg in two lines, part 1
    elif data[0] == '!AIVDM' and data[1] == '2' and data[2] == '1':
        buff = data[5]

    # msg in two lines, part 2
    elif data[0] == '!AIVDM' and data[1] == '2' and data[2] == '2':
        if len(buff) == 56 and len(data[5]) == 15:
            total = buff + data[5]
            publish(ais.decode(total, 2))
            buff = ''
        else:
            publish('error', 'biz')
