#!/usr/bin/env python

import rospy
import battery_maestro
from std_msgs.msg import Float32


rospy.init_node('batterycheck')

# cree une liste avec les tensions de chaque batterie dans les cases 0 a 7
# et les courants deux par deux dans les cases 8 a 11


def formArray(battery):
    batList = battery.getState()
    for i in range(8):
        batList[i] = float(batList[i]) / 1023. * 9.5 * 4.85
    for i in range(4):
        batList[i + 8] = float(batList[i + 8]) / 1023. * 17. * 4.85
    return batList


def publer(pubvolt, pubamp, battery):
    batstate = formArray(battery)
    for i in range(8):
        pubvolt[i].publish(batstate[i])
    for i in range(4):
        pubamp[i].publish(batstate[i + 8])

pubvolt = [rospy.Publisher('battery/volt' + str(i),
                           Float32, queue_size=1) for i in range(1, 9)]


pubamp = [rospy.Publisher('battery/amp' + str(i),
                          Float32, queue_size=1) for i in range(1, 5)]
# pubvolt = [rospy.Publisher('battery/volt1', Float32, queue_size=1),
#            rospy.Publisher('battery/volt2', Float32, queue_size=1),
#            rospy.Publisher('battery/volt3', Float32, queue_size=1),
#            rospy.Publisher('battery/volt4', Float32, queue_size=1),
#            rospy.Publisher('battery/volt5', Float32, queue_size=1),
#            rospy.Publisher('battery/volt6', Float32, queue_size=1),
#            rospy.Publisher('battery/volt7', Float32, queue_size=1),
#            rospy.Publisher('battery/volt8', Float32, queue_size=1)]

# pubamp = [rospy.Publisher('battery/amp1', Float32, queue_size=1),
# rospy.Publisher('battery/amp2', Float32, queue_size=1),
# rospy.Publisher('battery/amp3', Float32, queue_size=1),
# rospy.Publisher('battery/amp4', Float32, queue_size=1)]


battery = battery_maestro.battChecker(0)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    publer(pubvolt, pubamp, battery)
    rate.sleep()
