#!/usr/bin/env  python
import rospy
from process.srv import behavior, behaviorResponse, behaviorRequest
from Behavior import Behavior


def add_behavior(request):
    global listB
    print request.info.type
    listB.append(Behavior(request.info))
    print 'nombres de behavior recu:', len(listB)
    return behaviorResponse('oui')


rospy.init_node('service_server')
listB = []
service = rospy.Service('behavior_manager', behavior, add_behavior)
rospy.spin()
